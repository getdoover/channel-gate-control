"""Unit tests for the safety-critical output and fault logic.

These bypass the pydoover framework (object.__new__ + stubbed config /
platform_iface) so we can exercise _write_outputs and _check_move_safety
directly. The output test in particular guards the exact class of bug where the
DO writer calls a wrong/nonexistent platform method - it must call the real
`set_do` and must never energise both solenoids.
"""

import time
from types import SimpleNamespace

import pytest
from control_harness import (
    CONTROL_APP_KEY,
    ENCODER_APP_KEY,
    FakeTagsManager,
    build_control_app,
)

from channel_gate_control.application import ChannelGateControlApplication as App


def _cfg(**kw):
    return SimpleNamespace(**{k: SimpleNamespace(value=v) for k, v in kw.items()})


class FakePlatform:
    def __init__(self, di_level=0):
        self.calls = []
        self.di_level = di_level  # what fetch_di returns for the top-limit poll

    async def set_do(self, do, value):  # the REAL pydoover method name
        self.calls.append((do, value))

    async def fetch_di(self, *di):
        return self.di_level


class FakeTag:
    """One writable tag, enough for the app's publish path."""

    def __init__(self, value=None):
        self.value = value

    async def set(self, value):
        self.value = value


def _app_with(platform, **cfg):
    app = object.__new__(App)
    app._raise_state = False
    app._lower_state = False
    app._pump_state = False
    app._top_limit_active = False
    app._height_offset = 0.0
    app._limit_calibrated = False
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


async def test_top_limit_blocks_raise_allows_lower():
    plat = FakePlatform()
    app = _app_with(plat, raise_do_pin=2, lower_do_pin=3, pump_do_pin=4, do_active_low=False)
    app._top_limit_active = True

    # Raise while on the top limit: everything stays off (pump too).
    await app._write_outputs(True, False)
    assert plat.calls[-1] == ([2, 3, 4], [0, 0, 0])
    assert not app._raise_state and not app._pump_state

    # Lowering off the limit is allowed - that's the recovery path.
    await app._write_outputs(False, True)
    assert plat.calls[-1] == ([2, 3, 4], [0, 1, 1])


#: The top limit prox's height on this gate - the calibration datum.
LIMIT_HEIGHT_MM = 520.0


def _limit_app(plat, **extra):
    extra.setdefault("estop_active_low", False)  # normally-open prox
    extra.setdefault("estop_height_mm", LIMIT_HEIGHT_MM)
    app = _app_with(
        plat,
        raise_do_pin=2, lower_do_pin=3, pump_do_pin=4,
        do_active_low=False, estop_di_pin=5,
        **extra,
    )
    app._moving = None
    app._idle_since = None
    app._move_started = 0.0
    app._stall_ref_t = 0.0
    app._stall_ref_h = 0.0
    app._fault = False
    app._fault_reason = ""
    app._status = "ready"
    return app


async def test_top_limit_edge_stops_but_never_latches_a_fault():
    plat = FakePlatform(di_level=1)  # NO prox: closed at the limit -> DI HIGH
    app = _limit_app(plat)
    app._moving = "raise"
    app._status = "raising"

    # di_value is passed as False on purpose: pydoover delivers the proto3
    # default on EVERY driver, so an active-HIGH (normally-open) prox would never
    # be seen if the callback trusted it. The level read is what must decide.
    await app._on_top_limit(5, False, 0.01, 1, "rising")
    assert app._top_limit_active
    assert not app._fault and app._fault_reason == ""  # a warning, not a trip
    assert plat.calls[-1] == ([2, 3, 4], [0, 0, 0])
    assert app._moving is None
    assert "top limit" in app._status

    # A spurious INACTIVE edge (noise) must NOT clear the block on its own -
    # only a confirmed level poll can. The edge callback engages, never releases.
    plat.di_level = 0
    await app._on_top_limit(5, False, 0.01, 2, "falling")
    assert app._top_limit_active


async def test_top_limit_poll_is_backstop_and_clears_on_confirmed_level():
    # Missed activating edge: the poll sees the active level and engages anyway.
    plat = FakePlatform(di_level=1)
    app = _limit_app(plat)
    app._limit_calibrated = True  # pretend this arrival was already re-zeroed
    await app._poll_top_limit()
    assert app._top_limit_active
    assert not app._fault

    # Gate lowered off the prox: a confirmed inactive read lifts the block by
    # itself (no operator Reset), and arms the next arrival to re-zero again.
    plat.di_level = 0
    await app._poll_top_limit()
    assert not app._top_limit_active
    assert not app._limit_calibrated

    # A failed/None read HOLDS the last state - never releases on a bad read.
    app._top_limit_active = True

    async def boom(*a):
        raise RuntimeError("io down")

    plat.fetch_di = boom
    await app._poll_top_limit()
    assert app._top_limit_active


async def test_top_limit_polarity_is_configurable():
    # Normally-open (default): HIGH is at-the-limit.
    plat = FakePlatform(di_level=1)
    app = _limit_app(plat)
    assert app._top_limit_level_active(1) and not app._top_limit_level_active(0)

    # Normally-closed: the line sits HIGH and drops LOW at the limit.
    app = _limit_app(FakePlatform(di_level=1), estop_active_low=True)
    assert app._top_limit_level_active(0) and not app._top_limit_level_active(1)
    await app._poll_top_limit()  # HIGH = clear of the target
    assert not app._top_limit_active
    app.platform_iface.di_level = 0
    await app._poll_top_limit()
    assert app._top_limit_active


async def test_top_limit_control_blocks_raise_allows_lower():
    plat = FakePlatform()
    app = _limit_app(plat, deadband_mm=5.0, hysteresis_mm=5.0, outputs_enabled=True)
    app._top_limit_active = True

    # Target ABOVE height while on the top limit: hold everything off, don't
    # begin a move, don't run stall detection.
    await app._control(height=900.0, target=1000.0, mode="auto", trust_issue=None)
    assert plat.calls[-1] == ([2, 3, 4], [0, 0, 0])
    assert app._moving is None
    assert "raise blocked" in app._status

    # Target BELOW height: the operator can still close the gate off the limit.
    await app._control(height=900.0, target=400.0, mode="auto", trust_issue=None)
    assert plat.calls[-1] == ([2, 3, 4], [0, 1, 1])  # lower + pump, raise off
    assert app._moving == "lower"
    # Started through _begin_move, so the move timers are real: a stale
    # _move_started would fault with a move timeout the moment the limit cleared
    # and normal control inherited this move.
    assert time.monotonic() - app._move_started < 1.0


def test_top_limit_rezeroes_the_height_to_the_datum():
    app = _limit_app(FakePlatform())
    app._moving = None

    # Gate arrives at the prox with the encoder reading 600 mm - 80 mm of drift,
    # since the prox IS 520 mm. The reading is offset to come out at the datum.
    app._zero_at_top_limit(600.0)
    assert app._height_offset == -80.0
    assert app._limit_calibrated
    assert 600.0 + app._height_offset == LIMIT_HEIGHT_MM

    # Drift the other way: an encoder reading low is corrected upward.
    app._limit_calibrated = False
    app._zero_at_top_limit(475.0)
    assert app._height_offset == 45.0
    assert 475.0 + app._height_offset == LIMIT_HEIGHT_MM


def test_top_limit_datum_is_configurable():
    app = _limit_app(FakePlatform(), estop_height_mm=1000.0)
    app._moving = "raise"
    app._stall_ref_h = 0.0

    app._zero_at_top_limit(980.0)
    assert app._height_offset == 20.0
    assert 980.0 + app._height_offset == 1000.0
    # Mid-move, the calibrated height just stepped: the stall reference has to
    # move with it or the step reads as a jam / a wrong-way move.
    assert app._stall_ref_h == 1000.0
    assert time.monotonic() - app._stall_ref_t < 1.0


def test_read_height_reads_source_app_tag():
    app = object.__new__(App)
    app.config = _cfg(height_tag_name="Height", height_app_key="channel_gate_encoder_1")
    seen = {}

    def get_tag(name, app_key=None):
        seen["args"] = (name, app_key)
        return 412.5

    app.get_tag = get_tag
    assert app._read_raw_height() == 412.5
    # Must read the ENCODER INSTALL's namespace, never our own.
    assert seen["args"] == ("Height", "channel_gate_encoder_1")


def test_read_height_bad_values_are_none():
    app = object.__new__(App)
    app.config = _cfg(height_tag_name="Height", height_app_key="enc")
    app.get_tag = lambda name, app_key=None: None
    assert app._read_raw_height() is None
    app.get_tag = lambda name, app_key=None: "not-a-number"
    assert app._read_raw_height() is None


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


async def _loop_app(plat, height, target=0.0, **cfg):
    """A real app driven through ``main_loop``, over an in-memory tag bus.

    The tests above poke individual methods; the re-zero is a main_loop-level
    behaviour (it needs the encoder's height, which the DI callback doesn't have),
    so this exercises the loop itself. Only the platform is a stub.
    """
    bus = FakeTagsManager()
    if height is not None:
        await bus.set_tag("Height", height, app_key=ENCODER_APP_KEY)
    await bus.set_tag("Homed", True, app_key=ENCODER_APP_KEY)
    await bus.set_tag("Heartbeat", time.time(), app_key=ENCODER_APP_KEY)
    app = build_control_app(
        plat, bus, target=target, mode="auto", estop_di_pin=5, **cfg
    )
    return app, bus


async def test_top_limit_rezeroes_once_per_arrival():
    plat = FakePlatform(di_level=1)  # gate already sitting on the NO prox
    app, bus = await _loop_app(plat, 600.0, target=LIMIT_HEIGHT_MM)

    await app.main_loop()
    assert app._top_limit_active
    assert not bus.get_tag("Fault", app_key=CONTROL_APP_KEY)
    assert app._height_offset == pytest.approx(-80.0)
    assert bus.get_tag("GateHeight", app_key=CONTROL_APP_KEY) == pytest.approx(
        LIMIT_HEIGHT_MM
    )
    assert bus.get_tag("HeightOffset", app_key=CONTROL_APP_KEY) == pytest.approx(-80.0)
    assert bus.get_tag("TopLimitActive", app_key=CONTROL_APP_KEY)

    # The gate creeps 10 mm while still on the prox. Re-zeroing again would peg
    # the reading at the datum and make that movement invisible.
    await bus.set_tag("Height", 610.0, app_key=ENCODER_APP_KEY)
    await app.main_loop()
    assert app._height_offset == pytest.approx(-80.0)
    assert bus.get_tag("GateHeight", app_key=CONTROL_APP_KEY) == pytest.approx(
        LIMIT_HEIGHT_MM + 10.0
    )

    # Off the prox and back on again: that IS a fresh calibration point.
    plat.di_level = 0
    await app.main_loop()
    assert not app._top_limit_active
    plat.di_level = 1
    await bus.set_tag("Height", 640.0, app_key=ENCODER_APP_KEY)
    await app.main_loop()
    assert app._height_offset == pytest.approx(-120.0)
    assert bus.get_tag("GateHeight", app_key=CONTROL_APP_KEY) == pytest.approx(
        LIMIT_HEIGHT_MM
    )


async def test_top_limit_rezero_waits_for_a_height_to_arrive():
    """The limit can land cycles before the encoder publishes anything.

    On a cold boot with the gate already on the prox, the height tag can be empty
    for several cycles. The calibration has to keep waiting for it rather than
    being consumed by the arrival it couldn't act on.
    """
    plat = FakePlatform(di_level=1)
    app, bus = await _loop_app(plat, None)  # no Height tag published yet

    await app.main_loop()
    assert app._top_limit_active  # the limit itself is honoured immediately
    assert not app._limit_calibrated
    assert app._height_offset == 0.0

    await bus.set_tag("Height", 812.0, app_key=ENCODER_APP_KEY)
    await app.main_loop()
    assert app._limit_calibrated
    assert app._height_offset == pytest.approx(LIMIT_HEIGHT_MM - 812.0)


async def test_top_limit_does_not_hold_the_outputs_after_it_clears():
    """No latch: once the gate is off the prox, Auto control resumes by itself."""
    plat = FakePlatform(di_level=1)
    app, bus = await _loop_app(plat, 1000.0, target=0.0, estop_height_mm=1000.0)

    await app.main_loop()
    assert app._top_limit_active
    assert app._height_offset == pytest.approx(0.0)  # datum matches the encoder
    # Target is 0 and the gate is at 1000: lowering is allowed on the limit.
    assert plat.calls[-1] == ([2, 3, 4], [0, 1, 1])

    # Gate comes off the prox on its way down. Nothing to reset - it keeps going.
    plat.di_level = 0
    await bus.set_tag("Height", 900.0, app_key=ENCODER_APP_KEY)
    await app.main_loop()
    assert not app._top_limit_active
    assert not app._fault
    assert app._moving == "lower"
    assert plat.calls[-1] == ([2, 3, 4], [0, 1, 1])


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
