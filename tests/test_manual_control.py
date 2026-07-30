"""Tests for local manual control: a momentary switch wired to an analog input.

The switch puts 12 V on the pin while it is held, and the gate jogs that way
until it is let go. Two things make this worth testing hard rather than trusting
to the state machine:

  * a RELEASE has no pulse stream behind it (one VI threshold per pin, spent on
    the press), so the level poll in ``main_loop`` is the only thing that stops
    the gate, and
  * the manual gate sits ABOVE the fault latch, so it deliberately drives in
    situations - dead encoder, latched stall, Hold mode - where every other path
    in this app holds the outputs off.

These drive the real ``main_loop`` over the in-memory tag bus; only the platform
is a stub (``FakePlatform``, shared with ``test_control.py``).
"""

import logging
import time

import pytest
from control_harness import (
    CONTROL_APP_KEY,
    ENCODER_APP_KEY,
    FakeTagsManager,
    build_control_app,
)

from .test_control import FakePlatform

#: Analog inputs the two local switches are wired to (only AI0/AI1 can do this).
RAISE_AI = 0
LOWER_AI = 1

#: Solenoid/pump DO pins, as the harness configures them.
OFF = ([2, 3, 4], [0, 0, 0])
RAISING = ([2, 3, 4], [1, 0, 1])
LOWERING = ([2, 3, 4], [0, 1, 1])

PRESSED_V = 12.0
RELEASED_V = 0.0


async def _manual_app(plat, height=400.0, target=400.0, mode="hold", homed=True, **cfg):
    """A real controller with both local switches wired, on a live tag bus."""
    bus = FakeTagsManager()
    if height is not None:
        await bus.set_tag("Height", height, app_key=ENCODER_APP_KEY)
    await bus.set_tag("Homed", homed, app_key=ENCODER_APP_KEY)
    await bus.set_tag("Heartbeat", time.time(), app_key=ENCODER_APP_KEY)
    cfg.setdefault("manual_raise_ai_pin", RAISE_AI)
    cfg.setdefault("manual_lower_ai_pin", LOWER_AI)
    app = build_control_app(plat, bus, target=target, mode=mode, **cfg)
    return app, bus


# ----------------------------------------------------------------------
# Registration
# ----------------------------------------------------------------------
def test_the_switch_pins_default_to_the_standard_wiring():
    """Local control is armed by default on AI0 (raise) / AI1 (lower).

    Every gate ships with the switch fitted, so the standard wiring is the
    default - and pydoover backfills schema defaults for keys an already-
    deployed config doesn't carry, which arms local control on upgrade too.
    The flip side is that a site with anything ELSE on AI0/AI1 must set the
    pins to null explicitly: an armed pin drives the gate above the fault
    latch. That trade was made knowingly; None stays expressible as "no
    switch fitted".
    """
    from channel_gate_control.app_config import ChannelGateControlConfig

    cfg = ChannelGateControlConfig()
    assert cfg.manual_raise_ai_pin.default == 0
    assert cfg.manual_lower_ai_pin.default == 1
    for element in (cfg.manual_raise_ai_pin, cfg.manual_lower_ai_pin):
        # Only AI0/AI1 can do the voltage-step press detection.
        assert element.maximum == 1


async def test_setup_arms_a_press_detector_on_each_switch_pin():
    plat = FakePlatform()
    app, _ = await _manual_app(plat)
    await app.setup()

    # "VI" makes the pin argument an ANALOG selector; "+6" is the upward
    # sample-to-sample step of the 0 -> 12 V press; "@0.1" the firmware poll rate.
    assert [(c.pin, c.edge) for c in plat.counters] == [
        (RAISE_AI, "VI+6@0.1"),
        (LOWER_AI, "VI+6@0.1"),
    ]
    assert all(c.callback == app._on_manual_pulse for c in plat.counters)
    # Held so the counters can't be garbage-collected out from under the stream.
    assert len(app._manual_counters) == 2


async def test_default_poll_rate_emits_the_legacy_bare_edge_string():
    """0.4 s is the firmware default, and old builds crash on an "@" suffix."""
    plat = FakePlatform()
    app, _ = await _manual_app(plat, manual_poll_s=0.4)
    await app.setup()
    assert [c.edge for c in plat.counters] == ["VI+6", "VI+6"]

    # A non-default threshold formats without trailing zeros.
    plat = FakePlatform()
    app, _ = await _manual_app(plat, manual_threshold_v=4.5, manual_poll_s=0.25)
    await app.setup()
    assert [c.edge for c in plat.counters] == ["VI+4.5@0.25", "VI+4.5@0.25"]


async def test_unset_pins_disable_manual_control_entirely():
    """Null pins mean "no switch fitted": nothing armed, nothing polled."""
    plat = FakePlatform()
    app, _ = await _manual_app(plat, manual_raise_ai_pin=None, manual_lower_ai_pin=None)
    await app.setup()
    assert plat.counters == []

    # Nothing registered and nothing polled: the analog inputs are never touched.
    await app.main_loop()
    assert plat.ai_reads == []


async def test_one_configured_pin_is_read_as_a_bare_float():
    """A single pin makes fetch_ai answer with a scalar, not a one-item list."""
    plat = FakePlatform(ai_levels={RAISE_AI: PRESSED_V})
    app, _ = await _manual_app(plat, manual_lower_ai_pin=None)
    await app.setup()
    assert [(c.pin, c.edge) for c in plat.counters] == [(RAISE_AI, "VI+6@0.1")]

    await app.main_loop()
    assert plat.ai_reads == [(RAISE_AI,)]
    assert plat.calls[-1] == RAISING
    assert app._manual_raise_active and not app._manual_lower_active


async def test_an_unreadable_threshold_falls_back_to_the_schema_default():
    """Never to 0 V - that would read a released switch as held down."""
    plat = FakePlatform(ai_levels={RAISE_AI: PRESSED_V})
    app, _ = await _manual_app(plat, manual_threshold_v=None)
    await app.setup()
    assert [c.edge for c in plat.counters] == ["VI+6@0.1", "VI+6@0.1"]

    await app.main_loop()
    assert plat.calls[-1] == RAISING

    # And a released switch still reads released, which a 0 V threshold could
    # never do - every non-negative level would clear it.
    plat.ai_levels[RAISE_AI] = RELEASED_V
    await app.main_loop()
    assert not app._manual_raise_active
    assert plat.calls[-1] == OFF


@pytest.mark.parametrize("threshold", [0.0, -1.0])
async def test_a_non_positive_threshold_disables_manual_control(threshold, caplog):
    """0 V can't mean "always pressed", so it means "local control off"."""
    plat = FakePlatform(ai_levels={RAISE_AI: PRESSED_V, LOWER_AI: PRESSED_V})
    app, _ = await _manual_app(plat, manual_threshold_v=threshold)

    with caplog.at_level(logging.WARNING):
        await app.setup()
    assert plat.counters == []
    assert "DISABLED" in caplog.text  # said once, at setup: an operator must fix it

    # Both switches are held down and nothing reads them, nothing latches, and
    # nothing drives.
    await app.main_loop()
    assert plat.ai_reads == []
    assert not app._manual_raise_active and not app._manual_lower_active
    assert plat.calls[-1] == OFF


async def test_both_pins_are_read_in_one_transaction():
    plat = FakePlatform()
    app, _ = await _manual_app(plat)
    await app.main_loop()
    assert plat.ai_reads == [(RAISE_AI, LOWER_AI)]


# ----------------------------------------------------------------------
# Jogging
# ----------------------------------------------------------------------
async def test_press_raise_jogs_and_release_stops():
    plat = FakePlatform(ai_levels={RAISE_AI: PRESSED_V})
    app, bus = await _manual_app(plat)

    await app.main_loop()
    assert plat.calls[-1] == RAISING
    assert "manual raise" in app._status
    assert bus.get_tag("ManualRaise", app_key=CONTROL_APP_KEY)
    assert not bus.get_tag("ManualLower", app_key=CONTROL_APP_KEY)

    # Let go: the level poll is the only release detector there is, so the gate
    # stops within one control period.
    plat.ai_levels[RAISE_AI] = RELEASED_V
    await app.main_loop()
    assert plat.calls[-1] == OFF
    assert not bus.get_tag("ManualRaise", app_key=CONTROL_APP_KEY)


async def test_press_lower_jogs_down():
    plat = FakePlatform(ai_levels={LOWER_AI: PRESSED_V})
    app, bus = await _manual_app(plat)

    await app.main_loop()
    assert plat.calls[-1] == LOWERING
    assert "manual lower" in app._status
    assert bus.get_tag("ManualLower", app_key=CONTROL_APP_KEY)


async def test_both_switches_held_drives_nothing(caplog):
    plat = FakePlatform(ai_levels={RAISE_AI: PRESSED_V, LOWER_AI: PRESSED_V})
    app, _ = await _manual_app(plat)

    with caplog.at_level(logging.ERROR):
        await app.main_loop()
    assert plat.calls[-1] == OFF
    assert "both switches" in app._status
    # Resolved before the output choke point, so its last-resort interlock
    # assertion is never reached - an operator holding both is not a code bug.
    assert "Interlock violation" not in caplog.text


async def test_manual_works_with_no_height_and_an_unhomed_encoder():
    """The situations auto control refuses to move in are the point of this."""
    plat = FakePlatform(ai_levels={RAISE_AI: PRESSED_V})
    app, _ = await _manual_app(plat, height=None, homed=False, mode="hold")

    await app.main_loop()
    assert plat.calls[-1] == RAISING
    assert not app._fault  # nothing latched on the way through

    # Mode is irrelevant too: Hold only means "don't chase the setpoint".
    app.ui.mode.value = "auto"
    await app.main_loop()
    assert plat.calls[-1] == RAISING


async def test_manual_overrides_an_auto_move_and_auto_resumes_after_release():
    plat = FakePlatform()
    app, _ = await _manual_app(plat, height=400.0, target=1000.0, mode="auto")

    # Auto is raising toward the target.
    await app.main_loop()
    assert app._moving == "raise"
    assert plat.calls[-1] == RAISING

    # The operator at the gate wants it down. Manual wins, and the auto move's
    # bookkeeping is dropped so its timers don't carry over.
    plat.ai_levels[LOWER_AI] = PRESSED_V
    await app.main_loop()
    assert plat.calls[-1] == LOWERING
    assert app._moving is None
    assert "manual lower" in app._status

    # Released: auto picks the target back up on the very next pass, no reset,
    # no extra state.
    plat.ai_levels[LOWER_AI] = RELEASED_V
    await app.main_loop()
    assert app._moving == "raise"
    assert plat.calls[-1] == RAISING
    assert not app._fault


async def test_a_short_jog_does_not_hand_a_stale_move_clock_back_to_auto():
    """A manual jog ends the auto move's episode - it is never a continuation.

    ``_begin_move`` treats a re-engagement within ``_EPISODE_SETTLE_S`` as the
    same movement episode and keeps the old start time, so the move-timeout can
    accrue across hunting. A manual jog moves the gate an arbitrary distance,
    though, so the move auto starts afterwards is a fresh one - inheriting the
    abandoned move's clock would fault it on a timeout it never earned.
    """
    plat = FakePlatform()
    app, _ = await _manual_app(
        plat, height=400.0, target=1000.0, mode="auto", move_timeout_s=5.0
    )

    # Auto is raising, and has been energised for far longer than the timeout.
    await app.main_loop()
    assert app._moving == "raise"
    app._move_started -= 60.0

    # The operator jogs it down, briefly - well inside _EPISODE_SETTLE_S, which is
    # exactly the window that made the resumed move look like hunting.
    plat.ai_levels[LOWER_AI] = PRESSED_V
    await app.main_loop()
    assert plat.calls[-1] == LOWERING
    assert app._idle_since is None  # no episode to continue

    # Released: auto starts a FRESH move on a fresh clock.
    plat.ai_levels[LOWER_AI] = RELEASED_V
    await app.main_loop()
    assert app._moving == "raise"
    assert time.monotonic() - app._move_started < 1.0

    # ...and the next pass, the one that actually tests the move clock, is clean.
    await app.main_loop()
    assert not app._fault
    assert plat.calls[-1] == RAISING


async def test_top_limit_blocks_manual_raise_but_not_manual_lower():
    plat = FakePlatform(di_level=1, ai_levels={RAISE_AI: PRESSED_V})  # NO prox active
    app, _ = await _manual_app(plat, height=520.0, estop_di_pin=5)

    await app.main_loop()
    assert app._top_limit_active
    # Blocked here rather than at _write_outputs, so the pump is never energised
    # for a move that can't happen, and the status says why.
    assert plat.calls[-1] == OFF
    assert "manual raise blocked" in app._status

    # Lowering off the limit stays available - that is always the recovery path.
    plat.ai_levels[RAISE_AI] = RELEASED_V
    plat.ai_levels[LOWER_AI] = PRESSED_V
    await app.main_loop()
    assert plat.calls[-1] == LOWERING
    assert "manual lower" in app._status


async def test_manual_jogs_through_a_latched_fault_without_clearing_it():
    plat = FakePlatform(ai_levels={LOWER_AI: PRESSED_V})
    app, bus = await _manual_app(plat, height=400.0, target=1000.0, mode="auto")
    app._trip("gate not moving as commanded")

    await app.main_loop()
    assert plat.calls[-1] == LOWERING
    assert app._fault  # the latch is untouched; only Reset clears it

    # Released, auto is still held off by the fault - manual was a way to move
    # the gate, not a way around the trip.
    plat.ai_levels[LOWER_AI] = RELEASED_V
    await app.main_loop()
    assert plat.calls[-1] == OFF
    assert app._fault
    assert bus.get_tag("Fault", app_key=CONTROL_APP_KEY)
    assert "FAULT" in app._status


async def test_master_enable_outranks_manual():
    plat = FakePlatform(ai_levels={RAISE_AI: PRESSED_V})
    app, _ = await _manual_app(plat, outputs_enabled=False)

    await app.main_loop()
    assert plat.calls[-1] == OFF
    assert app._status == "outputs disabled"

    # The fast path respects it too - it must not sneak an output past the loop.
    before = len(plat.calls)
    await app._on_manual_pulse(RAISE_AI, False, 0.1, 1, "VI+6@0.1")
    assert all(call == OFF for call in plat.calls[before:])


# ----------------------------------------------------------------------
# Fail-safe reads
# ----------------------------------------------------------------------
@pytest.mark.parametrize("failure", ["raises", "none"])
async def test_a_bad_read_while_held_releases_the_switches(failure):
    """Fail-safe direction is RELEASED, the opposite of the top limit's hold.

    The top limit blocks an output, so an unknown reading must keep blocking.
    These switches drive one, so an unknown reading must stop driving.
    """
    plat = FakePlatform(ai_levels={RAISE_AI: PRESSED_V})
    app, _ = await _manual_app(plat)

    await app.main_loop()
    assert plat.calls[-1] == RAISING

    if failure == "raises":

        async def fetch_ai(*ai):
            raise RuntimeError("io down")
    else:

        async def fetch_ai(*ai):
            return None

    plat.fetch_ai = fetch_ai
    await app.main_loop()
    assert not app._manual_raise_active and not app._manual_lower_active
    assert plat.calls[-1] == OFF


async def test_a_level_list_that_does_not_match_the_pins_releases():
    """A short list can't be attributed to pins, so it isn't zipped at all.

    Zipping two directions against one level would read the RAISE pin's voltage
    as the LOWER switch's state - a misattributed level on an input that drives
    the gate. It counts as a failed read instead.
    """
    plat = FakePlatform(ai_levels={RAISE_AI: PRESSED_V})
    app, _ = await _manual_app(plat)

    await app.main_loop()
    assert plat.calls[-1] == RAISING

    async def fetch_ai(*ai):
        return [PRESSED_V]  # one level for the two pins asked for

    plat.fetch_ai = fetch_ai
    await app.main_loop()
    assert not app._manual_raise_active and not app._manual_lower_active
    assert plat.calls[-1] == OFF


async def test_clearing_the_pins_at_runtime_drops_a_held_switch():
    plat = FakePlatform(ai_levels={RAISE_AI: PRESSED_V})
    app, _ = await _manual_app(plat)

    await app.main_loop()
    assert app._manual_raise_active
    assert plat.calls[-1] == RAISING

    # A config update clears the pins while the switch still reads pressed. It is
    # the flag, not the pin, that drives the gate - so the flag has to drop too.
    app.config.manual_raise_ai_pin.value = None
    app.config.manual_lower_ai_pin.value = None
    await app.main_loop()
    assert not app._manual_raise_active and not app._manual_lower_active
    assert plat.calls[-1] == OFF


# ----------------------------------------------------------------------
# Fast path
# ----------------------------------------------------------------------
async def test_press_pulse_drives_without_waiting_for_the_control_period():
    plat = FakePlatform(ai_levels={RAISE_AI: PRESSED_V})
    app, _ = await _manual_app(plat)

    # di_value is False on purpose: pydoover delivers the proto3 default on every
    # driver, and a VI payload carries no analog level at all, so the callback
    # has to re-read the levels itself.
    await app._on_manual_pulse(RAISE_AI, False, 0.1, 1, "VI+6@0.1")
    assert plat.calls == [RAISING]  # no main_loop ran
    assert app._manual_raise_active


async def test_a_bouncing_pulse_with_nothing_held_does_nothing():
    """Stopping belongs to the loop, so the fast path can only ever start a jog."""
    plat = FakePlatform()  # both inputs at 0 V: already released
    app, _ = await _manual_app(plat)

    await app._on_manual_pulse(RAISE_AI, False, 0.1, 1, "VI+6@0.1")
    assert plat.calls == []
    assert not app._manual_raise_active


async def test_the_fast_path_never_raises_onto_an_unpolled_top_limit():
    """The counters are armed in setup(), before main_loop has seen the prox.

    So the fast path polls the limit itself before applying a raise: the
    never-raise-onto-the-prox invariant holds from the first pulse, not from the
    first control period.
    """
    plat = FakePlatform(di_level=1, ai_levels={RAISE_AI: PRESSED_V})  # NO prox active
    app, _ = await _manual_app(plat, height=520.0, estop_di_pin=5)
    await app.setup()  # listeners armed; main_loop has NOT run

    await app._on_manual_pulse(RAISE_AI, False, 0.1, 1, "VI+6@0.1")
    assert app._top_limit_active
    assert RAISING not in plat.calls
    assert plat.calls[-1] == OFF
    assert "manual raise blocked" in app._status


# ----------------------------------------------------------------------
# Publishing
# ----------------------------------------------------------------------
async def test_publish_reports_the_switch_states():
    plat = FakePlatform()
    app, bus = await _manual_app(plat)

    app._manual_raise_active = True
    app._manual_lower_active = False
    await app._publish(400.0, 400.0, "hold", None)
    assert bus.get_tag("ManualRaise", app_key=CONTROL_APP_KEY) is True
    assert bus.get_tag("ManualLower", app_key=CONTROL_APP_KEY) is False

    app._manual_raise_active = False
    app._manual_lower_active = True
    await app._publish(400.0, 400.0, "hold", None)
    assert bus.get_tag("ManualRaise", app_key=CONTROL_APP_KEY) is False
    assert bus.get_tag("ManualLower", app_key=CONTROL_APP_KEY) is True
