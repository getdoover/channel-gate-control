"""End-to-end: the encoder app and the controller, talking over tags, on one rig.

``test_control.py`` unit-tests the controller's safety logic against stubs. This
module runs the whole chain with nothing stubbed out but the hardware:

    controller writes DO -> hydraulic rig integrates height -> toothed target
    -> two prox sensors -> pulse callbacks -> encoder decodes position
    -> encoder publishes Height tag -> controller reads it -> controller stops

Both apps share one ``PlatformInterfaceSim`` (as they would share one doovit's IO)
and one in-memory tag bus keyed by ``app_key`` (as they would share the cloud), so
the inter-app read is the real ``get_tag(name, app_key=...)`` path.

The encoder now captures RISING edges only, so one decoded count is half a tooth
cycle: on the same 4 mm pitch target ``mm_per_count`` is 2.0, double the old
both-edge value, and the position granularity is 2 mm against a 5 mm deadband.

The rig runs at 60 mm/s, which at 2 mm per count is 30 counts/s -- **15 rising
edges/s on each prox sensor**, the corrected load the encoder has to survive while
the controller closes the loop around it. (The previous 30 mm/s at 1 mm/count was
30 mixed-polarity edges/s, i.e. only 7.5 rising/s per sensor.)

These tests skip unless the channel-gate-encoder repo is checked out alongside
this one (see ``conftest.py``); they are marked ``interop`` so they can be
deselected explicitly.
"""

import asyncio
import time

import pytest
from control_harness import (
    CONTROL_APP_KEY,
    ENCODER_APP_KEY,
    FakeTagsManager,
    build_control_app,
)
from gate_rig import HydraulicGateRig

from .conftest import ENCODER_REPO

pytestmark = [
    pytest.mark.interop,
    pytest.mark.skipif(
        ENCODER_REPO is None,
        reason="channel-gate-encoder repo not checked out alongside this one",
    ),
]

RAISE_PIN, LOWER_PIN, PUMP_PIN = 2, 3, 4
A_PIN, B_PIN = 0, 1
TOP_LIMIT_PIN = 5
#: 60 mm/s at 2.0 mm per count = 30 counts/s = 15 rising edges/s per sensor.
RATE_MM_S = 60.0
MM_PER_COUNT = 2.0
RISING_HZ_PER_SENSOR = RATE_MM_S / (2 * MM_PER_COUNT)

#: Counts of position error each direction reversal costs the rising-only encoder.
#: Measured in the encoder repo (``test_pulse_fidelity.REVERSAL_MISSIGN_COUNTS``):
#: the turnaround reads as a same-channel repeat, which carries no direction
#: information at all, so that edge is signed with the OLD direction -- 1 edge x 2
#: counts. A closed loop that overshoots and corrects pays this every time, and it
#: does not heal: here that is 2 x 2.0 = 4 mm of permanent position error per
#: correction, against a 5 mm deadband.
#:
#: It was 4 counts (8 mm) until the encoder started taking its period term from
#: the firmware's PIO-measured ``dt_secs`` instead of from callback arrival times;
#: that gave the edge AFTER the turnaround a usable period, so only the repeat
#: itself is now mis-signed.
REVERSAL_MISSIGN_COUNTS = 2
REVERSAL_MISSIGN_MM = REVERSAL_MISSIGN_COUNTS * MM_PER_COUNT


def _stopping_budget_mm(publish_interval: float, control_period: float = 0.25) -> float:
    """How far off target this rig can legitimately stop, in mm.

    Two independent terms, both first-order and both measured elsewhere:

    * **position staleness** -- the controller acts on a height that is up to
      ``publish_interval + control_period`` old, and the gate keeps moving during
      that window: ``RATE_MM_S x (publish_interval + control_period)``;
    * **encoder error** -- up to ``REVERSAL_MISSIGN_MM`` of permanent offset once
      the loop has reversed once to correct an overshoot.

    Derived rather than a magic number because it scales with gate speed, and this
    rig now runs at 60 mm/s (to put the encoder at the required 15 rising
    edges/s/sensor) where the old hard-coded 25/30 mm tolerances no longer hold.
    """
    return RATE_MM_S * (publish_interval + control_period) + REVERSAL_MISSIGN_MM


def _reversal_budget(encoder) -> int:
    """Counts of position error this run is allowed, from its own turnarounds.

    Each direction reversal shows up in the encoder as exactly one same-channel
    repeat (``missed``) and costs up to ``REVERSAL_MISSIGN_COUNTS``. How many
    reversals happen is the CONTROLLER's choice -- it depends on whether it
    overshot and had to correct, which varies run to run -- so the budget is
    derived from the observed turnaround count rather than fixed. A minimum of one
    is allowed so a clean single-direction run still has a non-zero budget for the
    edges held before the first direction measurement.
    """
    return REVERSAL_MISSIGN_COUNTS * max(1, encoder.decoder.missed)


class Rig:
    """Both apps, the shared platform, the tag bus and the physics, wired up."""

    def __init__(self, app_enc, app_ctl, platform, bus, gate, physics):
        self.encoder = app_enc
        self.control = app_ctl
        self.platform = platform
        self.bus = bus
        self.gate = gate
        self.physics = physics
        self._tasks: list[asyncio.Task] = []

    def start_apps(self):
        """Run each app's main loop at its own configured period, concurrently."""

        async def pump(app):
            while True:
                await app.main_loop()
                await asyncio.sleep(app.loop_target_period)

        for app in (self.encoder, self.control):
            self._tasks.append(asyncio.create_task(pump(app)))

    async def stop(self):
        for task in self._tasks:
            task.cancel()
        await self.platform.close()

    def published_height(self):
        return self.bus.get_tag("Height", app_key=ENCODER_APP_KEY)

    def status(self):
        return self.bus.get_tag("Status", app_key=CONTROL_APP_KEY)


async def build_rig(
    target: float,
    publish_interval: float = 0.5,
    start_mm: float = 0.0,
    top_limit_mm: float | None = None,
    **control_cfg,
) -> Rig:
    # Both of these come from the sibling channel-gate-encoder repo, which
    # conftest.py put on the path. The module names are deliberately distinct
    # between the two repos (encoder_harness / control_harness) so neither
    # shadows the other when both simulators/ directories are importable.
    from encoder_harness import build_encoder_app
    from platform_sim import PlatformInterfaceSim, QuadratureGateSim

    platform = PlatformInterfaceSim()
    bus = FakeTagsManager()

    # --- Encoder: same device, DI 0/1, publishing on its own timer ----------
    app_enc = build_encoder_app(
        platform,
        bus,
        app_key=ENCODER_APP_KEY,
        channel_a_pin=A_PIN,
        channel_b_pin=B_PIN,
        mm_per_count=MM_PER_COUNT,
        tag_publish_interval_s=publish_interval,
        display_refresh_period=publish_interval,
    )
    await app_enc.setup()
    platform.start()
    # The controller refuses to drive an unhomed encoder (require_homed), which is
    # correct -- so home it at the closed position, as an operator would.
    app_enc._do_home()

    # The rate here only paces gate.run_for(), which this rig never calls -- the
    # physics integrator drives gate.step() directly. Declared anyway so the rig's
    # nominal rate is on the record and matches the encoder's own tests.
    gate = QuadratureGateSim(
        platform,
        a_pin=A_PIN,
        b_pin=B_PIN,
        rising_edges_per_sensor_hz=RISING_HZ_PER_SENSOR,
    )
    await gate.seed()

    # --- Controller: same device, DO 2/3/4, reading the encoder's Height ----
    if top_limit_mm is not None:
        control_cfg.setdefault("estop_di_pin", TOP_LIMIT_PIN)
    app_ctl = build_control_app(
        platform,
        bus,
        target=target,
        mode="auto",
        height_app_key=ENCODER_APP_KEY,
        raise_do_pin=RAISE_PIN,
        lower_do_pin=LOWER_PIN,
        pump_do_pin=PUMP_PIN,
        **control_cfg,
    )
    await app_ctl.setup()

    physics = HydraulicGateRig(
        platform,
        gate,
        raise_pin=RAISE_PIN,
        lower_pin=LOWER_PIN,
        pump_pin=PUMP_PIN,
        rate_mm_s=RATE_MM_S,
        mm_per_count=MM_PER_COUNT,
        start_mm=start_mm,
        top_limit_pin=TOP_LIMIT_PIN if top_limit_mm is not None else None,
        top_limit_mm=top_limit_mm,
    )
    return Rig(app_enc, app_ctl, platform, bus, gate, physics)


def _report(name, rig, target, elapsed):
    enc, phys = rig.encoder, rig.physics
    text = (
        f"\n[{name}]\n"
        f"  target             : {target:.1f} mm\n"
        f"  true gate height   : {phys.height:.1f} mm\n"
        f"  encoder published  : {rig.published_height()} mm\n"
        f"  final error        : {phys.height - target:+.1f} mm\n"
        f"  peak height        : {phys.peak_height:.1f} mm "
        f"(overshoot {phys.peak_height - target:+.1f} mm)\n"
        f"  decoder count      : {enc.decoder.count} "
        f"(missed {enc.decoder.missed}, ambiguous {enc.decoder.ambiguous})\n"
        f"  true rising count  : {rig.gate.true_position}\n"
        f"  edges injected     : "
        f"{rig.platform.stats_for(A_PIN).injected + rig.platform.stats_for(B_PIN).injected}"
        f" all polarities, {rig.gate.rising_edges_emitted} rising\n"
        f"  solenoid reversals : {phys.reversals}\n"
        f"  drive time         : {phys.drive_time_s:.2f} s of {elapsed:.2f} s\n"
        f"  controller status  : {rig.status()!r}\n"
        f"  fault              : {rig.bus.get_tag('Fault', app_key=CONTROL_APP_KEY)} "
        f"{rig.bus.get_tag('FaultReason', app_key=CONTROL_APP_KEY)!r}\n"
    )
    print(text)
    return text


class TestEndToEndSeek:
    """The controller must drive the gate to the target and stop there."""

    async def test_reaches_target_and_stops(self):
        target = 120.0
        budget = _stopping_budget_mm(0.5)
        rig = await build_rig(target)
        rig.start_apps()
        start = time.monotonic()
        try:
            settled = await rig.physics.run_until(
                lambda: (
                    rig.control._moving is None
                    and abs(rig.physics.height - target) < budget
                    and rig.physics.drive_time_s > 1.0
                ),
                timeout_s=25.0,
            )
            # Let it sit, to prove it stays stopped rather than creeping.
            await rig.physics.run_until(lambda: False, timeout_s=2.0)
            elapsed = time.monotonic() - start
            text = _report("seek 120 mm, publish 500 ms", rig, target, elapsed)
            print(
                f"  => stopped {rig.physics.height - target:+.1f} mm from target, "
                f"against a {budget:.0f} mm budget "
                f"({RATE_MM_S:.0f} mm/s x 0.75 s staleness "
                f"+ {REVERSAL_MISSIGN_MM:.0f} mm encoder reversal error)\n"
            )

            assert settled, "controller never settled at the target" + text
            assert not rig.bus.get_tag("Fault", app_key=CONTROL_APP_KEY), text
            # No pulse was lost: every rising edge produced a callback and a
            # count. What error there is comes from the turnarounds the controller
            # chose to make, not from lost edges.
            assert rig.encoder.decoder.count == pytest.approx(
                rig.gate.true_position, abs=_reversal_budget(rig.encoder)
            ), text
            # Both solenoids must never have been energised together.
            for pins, values in rig.platform.do_writes:
                if RAISE_PIN in pins and LOWER_PIN in pins:
                    assert not (
                        values[pins.index(RAISE_PIN)] and values[pins.index(LOWER_PIN)]
                    ), f"interlock violated: {pins} {values}"
            # It stopped, and it stopped within the derived budget rather than
            # anywhere. The interesting number is the printed one, not this bound.
            assert rig.control._moving is None, text
            assert abs(rig.physics.height - target) < budget, text
        finally:
            await rig.stop()

    async def test_lowering_to_a_smaller_target_also_stops(self):
        """Direction independence: the same loop must work downwards."""
        rig = await build_rig(200.0)
        rig.start_apps()
        start = time.monotonic()
        try:
            await rig.physics.run_until(
                lambda: rig.control._moving is None and rig.physics.height > 150.0,
                timeout_s=25.0,
            )
            up_height = rig.physics.height
            reached_up = rig.encoder.decoder.count

            # Operator drags the slider back down.
            rig.control.ui.target.value = 60.0
            settled = await rig.physics.run_until(
                lambda: rig.control._moving is None and rig.physics.height < 100.0,
                timeout_s=25.0,
            )
            elapsed = time.monotonic() - start
            text = _report("lower 200 -> 60 mm", rig, 60.0, elapsed)
            print(
                f"  rose to {up_height:.1f} mm (count {reached_up}), "
                f"then lowered to {rig.physics.height:.1f} mm "
                f"(count {rig.encoder.decoder.count})\n"
            )

            assert settled, "controller never settled on the way down" + text
            assert not rig.bus.get_tag("Fault", app_key=CONTROL_APP_KEY), text
            # Absolute position tracked in BOTH directions, to within the cost of
            # the turnarounds themselves.
            assert rig.encoder.decoder.count == pytest.approx(
                rig.gate.true_position, abs=_reversal_budget(rig.encoder)
            ), text
            assert rig.encoder.decoder.count < reached_up, "count must come back down"
            assert rig.bus.get_tag("Direction", app_key=ENCODER_APP_KEY) in (
                "closing",
                "stopped",
            ), text
        finally:
            await rig.stop()


class TestPublishIntervalSetsStoppingAccuracy:
    """How accurately the gate can stop is bounded by how fresh the position is.

    The controller cannot stop the gate more precisely than the position it can
    see. Overshoot is roughly ``gate_speed x (publish_interval + control_period)``,
    so at 60 mm/s a 500 ms publish interval costs ~30 mm of position staleness on
    top of the 250 ms control period -- against a 5 mm deadband and a 2 mm position
    granularity. This measures it at two intervals so the trade-off is a number,
    not an opinion.
    """

    @pytest.mark.parametrize("publish_interval", [0.5, 0.1])
    async def test_overshoot_scales_with_publish_interval(self, publish_interval):
        target = 120.0
        rig = await build_rig(target, publish_interval=publish_interval)
        rig.start_apps()
        start = time.monotonic()
        try:
            await rig.physics.run_until(
                lambda: (
                    rig.control._moving is None
                    and rig.physics.drive_time_s > 1.0
                    and abs(rig.physics.height - target) < 40.0
                ),
                timeout_s=25.0,
            )
            await rig.physics.run_until(lambda: False, timeout_s=1.5)
            elapsed = time.monotonic() - start
            _report(f"publish {publish_interval * 1000:.0f} ms", rig, target, elapsed)

            overshoot = rig.physics.peak_height - target
            predicted = RATE_MM_S * (publish_interval + 0.25)
            drift = rig.encoder.decoder.count - rig.gate.true_position
            print(
                f"  => overshoot {overshoot:+.1f} mm; first-order prediction "
                f"{RATE_MM_S:.0f} mm/s x ({publish_interval:.2f} + 0.25) s "
                f"= {predicted:.1f} mm\n"
                f"  => encoder vs true rising count: {drift:+d} counts "
                f"= {drift * MM_PER_COUNT:+.1f} mm of PERMANENT position error "
                f"after {rig.encoder.decoder.missed} turnaround(s) "
                f"(up to {REVERSAL_MISSIGN_MM:.0f} mm each, and it does not heal)\n"
            )
            assert not rig.bus.get_tag("Fault", app_key=CONTROL_APP_KEY)
            # No lost edges. The only position error is the turnaround mis-sign,
            # budgeted against the number of turnarounds this run actually made.
            assert abs(drift) <= _reversal_budget(rig.encoder)
            # Overshoot must stay within the first-order prediction plus a margin
            # for the physics step and one extra control pass.
            assert overshoot <= predicted + RATE_MM_S * 0.35, (
                f"overshoot {overshoot:.1f} mm exceeded the staleness budget "
                f"{predicted:.1f} mm"
            )
        finally:
            await rig.stop()


class TestTopLimitProx:
    """The over-travel prox, end to end: block the raise, re-zero the height.

    The gate starts at a TRUE height of 100 mm with the encoder homed at 0, so the
    encoder reads 100 mm low - the state a gate is in when it was moved by hand,
    or homed at the wrong place. Nothing in the loop can know that until the gate
    touches the prox, which is the whole point of having one: the prox is at a
    known height, so arriving there is what pins the measurement to reality.
    """

    TRUE_START = 100.0
    LIMIT_MM = 300.0

    async def test_prox_blocks_raise_rezeros_height_and_still_lowers(self):
        rig = await build_rig(
            500.0,  # above the prox: the controller WANTS to keep raising
            publish_interval=0.1,
            start_mm=self.TRUE_START,
            top_limit_mm=self.LIMIT_MM,
            estop_height_mm=self.LIMIT_MM,  # the prox's real height = the datum
        )
        ctl = rig.control
        rig.start_apps()
        try:
            # --- Rise onto the prox --------------------------------------
            hit = await rig.physics.run_until(
                lambda: ctl._top_limit_active, timeout_s=25.0
            )
            error_before = ctl._height_offset  # 0 until the prox calibrates
            await rig.physics.run_until(lambda: ctl._limit_calibrated, timeout_s=5.0)
            # Let it sit on the limit, proving it does not keep driving up.
            await rig.physics.run_until(lambda: False, timeout_s=1.5)
            height_on_limit = rig.physics.height
            calibrated = rig.bus.get_tag("GateHeight", app_key=CONTROL_APP_KEY)
            offset = rig.bus.get_tag("HeightOffset", app_key=CONTROL_APP_KEY)
            budget = _stopping_budget_mm(0.1)

            print(
                f"\n[top limit prox]\n"
                f"  true height at prox : {height_on_limit:.1f} mm "
                f"(prox at {self.LIMIT_MM:.0f} mm)\n"
                f"  encoder published   : {rig.published_height()} mm "
                f"(homed {self.TRUE_START:.0f} mm low)\n"
                f"  offset before/after : {error_before:+.1f} / {offset:+.1f} mm\n"
                f"  calibrated height   : {calibrated:.1f} mm\n"
                f"  raise DO / pump DO  : "
                f"{rig.platform.do_levels.get(RAISE_PIN)} / "
                f"{rig.platform.do_levels.get(PUMP_PIN)}\n"
                f"  status              : {rig.status()!r}\n"
                f"  fault               : "
                f"{rig.bus.get_tag('Fault', app_key=CONTROL_APP_KEY)}\n"
            )

            assert hit, "the gate never reached the prox"
            # Raising is dead while the prox reads active, even though the
            # setpoint is still 200 mm above the gate.
            assert not rig.platform.do_levels.get(RAISE_PIN)
            assert not rig.platform.do_levels.get(PUMP_PIN)
            assert ctl._moving is None
            # A warning, not a trip: nothing latched, no Reset needed.
            assert not rig.bus.get_tag("Fault", app_key=CONTROL_APP_KEY)
            assert rig.bus.get_tag("FaultReason", app_key=CONTROL_APP_KEY) == ""
            assert rig.bus.get_tag("TopLimitActive", app_key=CONTROL_APP_KEY)
            # The re-zero corrected the encoder's ~100 mm error: the offset went
            # from nothing to about the error, and the height the controller acts
            # on now agrees with the real gate.
            assert offset == pytest.approx(self.TRUE_START, abs=budget)
            assert calibrated == pytest.approx(rig.physics.height, abs=budget)

            # --- Lowering is still available off the limit ----------------
            rig.control.ui.target.value = 150.0
            cleared = await rig.physics.run_until(
                lambda: not ctl._top_limit_active, timeout_s=25.0
            )
            settled = await rig.physics.run_until(
                lambda: ctl._moving is None and rig.physics.height < 250.0,
                timeout_s=25.0,
            )
            print(
                f"  lowered to          : {rig.physics.height:.1f} mm "
                f"(target 150 mm, budget {budget:.0f} mm)\n"
                f"  status              : {rig.status()!r}\n"
            )
            assert cleared, "the prox never released as the gate came down"
            assert settled, "the gate never settled after recovering off the prox"
            assert not rig.bus.get_tag("Fault", app_key=CONTROL_APP_KEY)
            # Stopped at the target in the CALIBRATED frame, which is now the real
            # one - so true height, not encoder height, is what lands on 150 mm.
            assert rig.physics.height == pytest.approx(150.0, abs=budget)
        finally:
            await rig.stop()


class TestSignalIntegrity:
    """The controller must refuse to drive on an untrustworthy position."""

    async def test_unhomed_encoder_holds_the_outputs(self):
        rig = await build_rig(120.0)
        # Undo the harness's homing: this is what a freshly-booted encoder looks
        # like, and its height is meaningless until it has homed.
        rig.encoder._homed = False
        await rig.encoder.tags.Homed.set(False)
        rig.start_apps()
        try:
            await rig.physics.run_until(lambda: False, timeout_s=2.0)
            status = rig.status()
            print(
                f"\n[unhomed encoder] status={status!r} "
                f"height_moved={rig.physics.height:.2f} mm "
                f"pump={rig.platform.do_levels.get(PUMP_PIN)}"
            )
            assert rig.physics.height == pytest.approx(0.0, abs=0.01), (
                "the gate must not move while the encoder is unhomed"
            )
            assert status == "encoder not homed"
            assert not rig.platform.do_levels.get(PUMP_PIN)
        finally:
            await rig.stop()

    async def test_stale_encoder_heartbeat_stops_the_gate(self):
        """A frozen encoder keeps its last Height in the tag cache forever."""
        rig = await build_rig(400.0)
        rig.start_apps()
        try:
            await rig.physics.run_until(
                lambda: rig.physics.height > 40.0, timeout_s=15.0
            )
            moving_height = rig.physics.height
            # Kill the encoder: no more publishes, so Heartbeat goes stale.
            rig._tasks[0].cancel()
            await rig.encoder.tags.Heartbeat.set(time.time() - 120.0)

            await rig.physics.run_until(
                lambda: rig.control._moving is None, timeout_s=5.0
            )
            frozen_height = rig.physics.height
            await rig.physics.run_until(lambda: False, timeout_s=1.0)

            print(
                f"\n[stale encoder] moving at {moving_height:.1f} mm -> stopped at "
                f"{frozen_height:.1f} mm, status={rig.status()!r}, "
                f"crept a further {rig.physics.height - frozen_height:.2f} mm"
            )
            assert rig.control._moving is None, "must stop driving blind"
            assert "stale" in str(rig.status())
            assert rig.physics.height == pytest.approx(frozen_height, abs=0.5)
        finally:
            await rig.stop()
