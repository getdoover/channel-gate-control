"""Unit tests for the safety-critical output and fault logic.

These bypass the pydoover framework (object.__new__ + stubbed config /
platform_iface) so we can exercise _write_outputs and _check_move_safety
directly. The output test in particular guards the exact class of bug where the
DO writer calls a wrong/nonexistent platform method - it must call the real
`set_do` and must never energise both solenoids.
"""

import time
from types import SimpleNamespace

from channel_gate_control.application import ChannelGateControlApplication as App


def _cfg(**kw):
    return SimpleNamespace(**{k: SimpleNamespace(value=v) for k, v in kw.items()})


class FakePlatform:
    def __init__(self, di_level=1):
        self.calls = []
        self.di_level = di_level  # what fetch_di returns for the e-stop poll

    async def set_do(self, do, value):  # the REAL pydoover method name
        self.calls.append((do, value))

    async def fetch_di(self, *di):
        return self.di_level


def _app_with(platform, **cfg):
    app = object.__new__(App)
    app._raise_state = False
    app._lower_state = False
    app._pump_state = False
    app._estop_active = False
    app.platform_iface = platform
    cfg.setdefault("pump_do_pin", None)
    app.config = _cfg(**cfg)
    return app


async def test_write_outputs_uses_real_api_and_interlocks():
    plat = FakePlatform()
    app = _app_with(plat, raise_do_pin=2, lower_do_pin=3, do_active_low=False)

    await app._write_outputs(True, False)  # raise
    assert plat.calls[-1] == ([2, 3], [1, 0])
    assert app._raise_state and not app._lower_state

    await app._write_outputs(False, True)  # lower
    assert plat.calls[-1] == ([2, 3], [0, 1])

    # Both requested must NEVER reach the hardware as both-on.
    await app._write_outputs(True, True)
    assert plat.calls[-1] == ([2, 3], [0, 0])
    assert not app._raise_state and not app._lower_state

    await app._write_outputs(False, False)  # stop
    assert plat.calls[-1] == ([2, 3], [0, 0])


async def test_write_outputs_active_low():
    plat = FakePlatform()
    app = _app_with(plat, raise_do_pin=2, lower_do_pin=3, do_active_low=True)
    # Energise raise on active-low wiring: raise pin driven LOW (0), lower HIGH (1).
    await app._write_outputs(True, False)
    assert plat.calls[-1] == ([2, 3], [0, 1])
    # Safe state (both de-energised) drives both HIGH on active-low.
    await app._write_outputs(False, False)
    assert plat.calls[-1] == ([2, 3], [1, 1])


async def test_pump_runs_with_either_solenoid_only():
    plat = FakePlatform()
    app = _app_with(plat, raise_do_pin=2, lower_do_pin=3, pump_do_pin=4, do_active_low=False)

    await app._write_outputs(True, False)  # raise -> pump on
    assert plat.calls[-1] == ([2, 3, 4], [1, 0, 1])
    assert app._pump_state

    await app._write_outputs(False, True)  # lower -> pump on
    assert plat.calls[-1] == ([2, 3, 4], [0, 1, 1])

    await app._write_outputs(False, False)  # stop -> pump off
    assert plat.calls[-1] == ([2, 3, 4], [0, 0, 0])
    assert not app._pump_state

    # Interlock violation collapses to all-off, pump included.
    await app._write_outputs(True, True)
    assert plat.calls[-1] == ([2, 3, 4], [0, 0, 0])

    # Active-low wiring: pump follows the same polarity.
    app = _app_with(plat, raise_do_pin=2, lower_do_pin=3, pump_do_pin=4, do_active_low=True)
    await app._write_outputs(True, False)
    assert plat.calls[-1] == ([2, 3, 4], [0, 1, 0])
    await app._write_outputs(False, False)
    assert plat.calls[-1] == ([2, 3, 4], [1, 1, 1])


async def test_estop_blocks_raise_allows_lower():
    plat = FakePlatform()
    app = _app_with(plat, raise_do_pin=2, lower_do_pin=3, pump_do_pin=4, do_active_low=False)
    app._estop_active = True

    # Raise while on the top limit: everything stays off (pump too).
    await app._write_outputs(True, False)
    assert plat.calls[-1] == ([2, 3, 4], [0, 0, 0])
    assert not app._raise_state and not app._pump_state

    # Lowering off the limit is allowed - that's the recovery path.
    await app._write_outputs(False, True)
    assert plat.calls[-1] == ([2, 3, 4], [0, 1, 1])


def _estop_app(plat, **extra):
    app = _app_with(
        plat,
        raise_do_pin=2, lower_do_pin=3, pump_do_pin=4,
        do_active_low=False, estop_di_pin=5, estop_active_low=True,
        **extra,
    )
    app._moving = None
    app._idle_since = None
    app._fault = False
    app._fault_reason = ""
    app._status = "ready"
    return app


async def test_estop_edge_trips_latches_fault_and_deenergises():
    plat = FakePlatform()
    app = _estop_app(plat)
    app._moving = "raise"
    app._status = "raising"

    # NC prox opens at the top limit -> DI falls to 0 -> immediate trip.
    await app._on_estop(5, 0, 0.01, 1, "falling")
    assert app._estop_active
    assert app._fault and "e-stop" in app._fault_reason
    assert plat.calls[-1] == ([2, 3, 4], [0, 0, 0])
    assert app._moving is None

    # A spurious INACTIVE edge (noise) must NOT clear the block on its own -
    # only a confirmed level poll can. Edge callback stays trip-only.
    await app._on_estop(5, 1, 0.01, 2, "rising")
    assert app._estop_active


async def test_estop_poll_is_backstop_and_clears_only_on_confirmed_level():
    # Missed activating edge: poll sees the active level and trips anyway.
    plat = FakePlatform(di_level=0)  # active-low asserted
    app = _estop_app(plat)
    await app._poll_estop()
    assert app._estop_active and app._fault

    # Gate lowered off the sensor: a confirmed inactive read clears the block
    # (the latched fault still needs an operator Reset).
    plat.di_level = 1
    await app._poll_estop()
    assert not app._estop_active
    assert app._fault  # still latched

    # A failed/None read HOLDS the last state - never releases on a bad read.
    app._estop_active = True

    async def boom(*a):
        raise RuntimeError("io down")

    plat.fetch_di = boom
    await app._poll_estop()
    assert app._estop_active


async def test_estop_control_blocks_raise_allows_lower_to_recover():
    plat = FakePlatform()
    app = _estop_app(plat, deadband_mm=5.0, hysteresis_mm=5.0, outputs_enabled=True)
    app._estop_active = True

    # Target ABOVE height while on the top limit: hold everything off, don't
    # begin a move, don't run stall detection.
    await app._control(height=900.0, target=1000.0, mode="auto", trust_issue=None)
    assert plat.calls[-1] == ([2, 3, 4], [0, 0, 0])
    assert app._moving is None
    assert "lower to recover" in app._status

    # Target BELOW height: lower to recover (moving away from the sensor).
    await app._control(height=900.0, target=400.0, mode="auto", trust_issue=None)
    assert plat.calls[-1] == ([2, 3, 4], [0, 1, 1])  # lower + pump, raise off
    assert app._moving == "lower"


def test_read_height_reads_source_app_tag():
    app = object.__new__(App)
    app.config = _cfg(height_tag_name="Height", height_app_key="channel_gate_encoder_1")
    seen = {}

    def get_tag(name, app_key=None):
        seen["args"] = (name, app_key)
        return 412.5

    app.get_tag = get_tag
    assert app._read_height() == 412.5
    # Must read the ENCODER INSTALL's namespace, never our own.
    assert seen["args"] == ("Height", "channel_gate_encoder_1")


def test_read_height_bad_values_are_none():
    app = object.__new__(App)
    app.config = _cfg(height_tag_name="Height", height_app_key="enc")
    app.get_tag = lambda name, app_key=None: None
    assert app._read_height() is None
    app.get_tag = lambda name, app_key=None: "not-a-number"
    assert app._read_height() is None


def _trust_app(tags, require_homed=True, heartbeat_timeout_s=15.0):
    app = object.__new__(App)
    app.config = _cfg(
        height_app_key="enc",
        require_homed=require_homed,
        heartbeat_timeout_s=heartbeat_timeout_s,
    )
    app.get_tag = lambda name, app_key=None: tags.get(name)
    return app


def test_trust_blocks_unhomed_encoder():
    app = _trust_app({"Homed": False, "Heartbeat": time.time()})
    assert app._height_trust_issue() == "encoder not homed"
    # Homed -> trusted.
    app = _trust_app({"Homed": True, "Heartbeat": time.time()})
    assert app._height_trust_issue() is None
    # require_homed off ignores the Homed tag entirely.
    app = _trust_app({"Homed": False, "Heartbeat": time.time()}, require_homed=False)
    assert app._height_trust_issue() is None


def test_trust_blocks_stale_or_missing_heartbeat():
    app = _trust_app({"Homed": True, "Heartbeat": time.time() - 60})
    issue = app._height_trust_issue()
    assert issue and "stale" in issue

    app = _trust_app({"Homed": True})  # no Heartbeat tag at all
    assert app._height_trust_issue() == "no encoder heartbeat"

    # Timeout of 0 disables the freshness check.
    app = _trust_app({"Homed": True}, heartbeat_timeout_s=0)
    assert app._height_trust_issue() is None


def test_stall_detects_wrong_way_and_passes_real_progress():
    app = object.__new__(App)
    app._moving = "raise"
    app.config = _cfg(move_timeout_s=30.0, stall_window_s=4.0, stall_min_progress_mm=3.0)

    now = time.monotonic()
    app._move_started = now  # well within timeout
    app._stall_ref_t = now - 10  # stall window has elapsed
    app._stall_ref_h = 500.0

    # Commanded to raise but gate went DOWN (reversed wiring) -> fault.
    reason = app._check_move_safety(480.0)
    assert reason and "not moving as commanded" in reason

    # Genuine upward progress -> no fault, and the window slides forward.
    app._move_started = time.monotonic()
    app._stall_ref_t = time.monotonic() - 10
    app._stall_ref_h = 500.0
    assert app._check_move_safety(520.0) is None
    assert app._stall_ref_h == 520.0
