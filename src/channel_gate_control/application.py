import logging
import time

from pydoover import ui
from pydoover.docker import Application

from .app_config import ChannelGateControlConfig
from .app_tags import ChannelGateControlTags
from .app_ui import ChannelGateControlUI

log = logging.getLogger(__name__)

# If the gate re-engages within this long of stopping at target, it's treated as
# the same movement "episode" (i.e. hunting/overshoot) rather than a fresh move,
# so the move-timeout keeps accumulating and eventually trips instead of the
# solenoids cycling forever. Rest longer than this and the next move starts clean.
_EPISODE_SETTLE_S = 3.0


class ChannelGateControlApplication(Application):
    """Closed-loop height control of a hydraulic channel gate.

    Reads the gate height published by the encoder app, compares it to the
    operator's slider setpoint, and drives one of two solenoid valves to close
    the error. Bang-bang control with a deadband (stop) + hysteresis (re-engage)
    so the valves don't chatter. Two hard safety properties:

      1. The raise and lower solenoids are written through a single choke point
         (`_write_outputs`) as one atomic transaction, so they can never be
         energised together.
      2. Any unsafe condition - encoder signal lost, move timeout, or the gate
         failing to move in the commanded direction (jam / dead encoder /
         reversed wiring) - de-energises both solenoids. Timeouts and stalls
         latch a fault that only an operator Reset clears.

    The top-limit prox is deliberately NOT one of those trips. It is a warning
    with teeth: while it reads active, raising is hard-blocked but lowering stays
    available (so the gate can always be closed), and the block lifts by itself
    once the gate comes off the sensor. Arriving at it also re-zeros the gate
    height - the prox is the calibration datum for this gate, so the height the
    encoder reports is anchored to it via `_height_offset`.
    """

    config_cls = ChannelGateControlConfig
    tags_cls = ChannelGateControlTags
    ui_cls = ChannelGateControlUI

    config: ChannelGateControlConfig
    tags: ChannelGateControlTags
    ui: ChannelGateControlUI

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._moving: str | None = None        # "raise" | "lower" | None
        self._move_started: float = 0.0
        self._stall_ref_t: float = 0.0
        self._stall_ref_h: float = 0.0
        self._fault: bool = False
        self._fault_reason: str = ""
        self._status: str = "starting"
        self._raise_state: bool = False
        self._lower_state: bool = False
        self._pump_state: bool = False
        self._top_limit_active: bool = False
        self._top_limit_counter = None  # kept referenced so it isn't garbage-collected
        # Calibration: gate height = encoder height + offset, re-zeroed whenever
        # the gate arrives at the top limit prox. `_limit_calibrated` tracks
        # whether THIS arrival has been used, so sitting on the sensor doesn't
        # re-zero every cycle (which would hide any creep).
        self._height_offset: float = 0.0
        self._limit_calibrated: bool = False
        self._idle_since: float | None = None  # when we last stopped (episode gap)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    async def setup(self):
        self.loop_target_period = float(self.config.control_period_s.value or 0.25)
        # Establish a known-safe output state before any control runs. Important
        # for active-low wiring, where an undriven pin reads as energised.
        await self._write_outputs(False, False)

        # Restore the height calibration across a restart. The encoder restores
        # its own count the same way, so its raw heights line up either side of a
        # restart and a stored offset stays meaningful; touching the prox
        # recalibrates regardless.
        try:
            persisted = self.tags.HeightOffset.value
            if persisted is not None:
                self._height_offset = float(persisted)
                log.info("Restored height calibration %+.1f mm", self._height_offset)
        except (TypeError, ValueError, AttributeError) as e:
            log.debug("No persisted height offset to restore: %s", e)

        # Top-limit prox. An input that gates an output must not depend on a
        # single edge being delivered, so detection is LEVEL-driven: main_loop
        # polls the DI every cycle (_poll_top_limit) as the guaranteed backstop.
        # The both-edge pulse counter is only the fast path for an immediate stop.
        # No initial read is needed here - the first main_loop poll runs before
        # any output is driven, so a gate already on the limit at boot blocks
        # raising (and gets re-zeroed) before it can be driven up.
        limit_pin = self._top_limit_pin()
        if limit_pin is not None:
            try:
                await self.platform_iface.set_di_config(limit_pin, debounce_ms=20)
            except Exception as e:
                log.debug("set_di_config(top limit %s) failed: %s", limit_pin, e)
            self._top_limit_counter = self.platform_iface.get_new_pulse_counter(
                limit_pin, edge="both", callback=self._on_top_limit
            )
            log.info("Top limit prox listener started on DI%s", limit_pin)

        self._status = "ready"

    async def on_shutdown_at(self, _dt):
        """Fail safe: drop both solenoids when the app is asked to stop."""
        try:
            await self._write_outputs(False, False)
        except Exception as e:  # best effort - never block shutdown
            log.warning("Failed to de-energise outputs on shutdown: %s", e)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    async def main_loop(self):
        # Poll the top limit FIRST, before any control decision or drive, so a
        # missed activating edge is caught within one control period.
        await self._poll_top_limit()

        raw_height = self._read_raw_height()
        # Re-zero while the gate sits on the prox. Done here rather than in the
        # DI callback because it needs the encoder's height, and that can arrive
        # several cycles after the limit does (app boot order, encoder still
        # starting) - so this keeps retrying until a height is available.
        if (
            self._top_limit_active
            and not self._limit_calibrated
            and raw_height is not None
        ):
            self._zero_at_top_limit(raw_height)

        height = None if raw_height is None else raw_height + self._height_offset
        target = self._read_target()
        mode = self._read_mode()
        trust_issue = self._height_trust_issue() if height is not None else None

        await self._control(height, target, mode, trust_issue)
        await self._publish(height, target, mode, trust_issue)

    async def _control(self, height, target, mode, trust_issue):
        # --- Safety / mode gates: any of these hold the outputs off -------
        if self._fault:
            await self._stop(f"FAULT: {self._fault_reason}")
            return
        if not self.config.outputs_enabled.value:
            await self._stop("outputs disabled")
            return
        if mode != "auto":
            await self._stop("hold")
            return
        if height is None:
            # Don't drive blind. Non-latching: resumes when the signal returns.
            await self._stop("no gate height signal")
            return
        if trust_issue:
            # Signal present but not trustworthy (unhomed / stale). Non-latching.
            await self._stop(trust_issue)
            return

        deadband = float(self.config.deadband_mm.value or 0.0)
        hysteresis = float(self.config.hysteresis_mm.value or 0.0)
        error = target - height

        if self._top_limit_active:
            # Gate is on the over-travel prox. Raising is hard-blocked (again at
            # the output choke point); lowering stays available so the operator
            # can always close the gate off the limit. No fault is latched - the
            # block lifts by itself once the gate comes off the sensor.
            # Stall/timeout detection is deliberately skipped here: with raise
            # suppressed, a raise request would otherwise fault with a misleading
            # 'gate not moving' reason.
            if error < -deadband:
                if self._moving != "lower":
                    # Through _begin_move so the move timers are real. Setting
                    # _moving directly would leave _move_started stale, and the
                    # move-timeout would then fire the moment the limit cleared
                    # and normal control inherited the move.
                    self._begin_move("lower", height)
                await self._drive("lower")
                self._status = "top limit - lowering (raise blocked)"
            else:
                await self._stop("top limit reached - raise blocked")
            return

        if self._moving is None:
            # Idle: only start once error clears the deadband + hysteresis band.
            if abs(error) <= deadband + hysteresis:
                await self._stop("at target")
                return
            self._begin_move("raise" if error > 0 else "lower", height)
        else:
            # Moving: stop as soon as we're inside the deadband...
            if abs(error) <= deadband:
                await self._stop("at target")
                return
            # ...or if we've reached/overshot and the error flipped direction.
            want = "raise" if error > 0 else "lower"
            if want != self._moving:
                await self._stop("target reached")
                return
            # Safety checks only apply once a move is underway.
            reason = self._check_move_safety(height)
            if reason:
                self._trip(reason)
                await self._stop(f"FAULT: {reason}")
                return

        await self._drive(self._moving)

    # ------------------------------------------------------------------
    # Movement helpers
    # ------------------------------------------------------------------
    def _begin_move(self, direction, height):
        now = time.monotonic()
        # A move that re-engages almost immediately after stopping at target is
        # overshoot/hunting, not a new move: keep the episode's start time so the
        # move-timeout accrues across the cycling instead of resetting each pass.
        fresh_episode = (
            self._idle_since is None or (now - self._idle_since) >= _EPISODE_SETTLE_S
        )
        if fresh_episode:
            self._move_started = now
        self._moving = direction
        self._idle_since = None
        self._stall_ref_t = now  # stall window is always per-move
        self._stall_ref_h = height
        log.info(
            "Begin %s from %.1f mm%s",
            direction,
            height,
            "" if fresh_episode else " (continuing episode)",
        )

    def _check_move_safety(self, height) -> str | None:
        """Return a fault reason if the move looks unsafe, else None."""
        now = time.monotonic()

        timeout = float(self.config.move_timeout_s.value or 0.0)
        if timeout > 0 and (now - self._move_started) > timeout:
            return f"move timeout - {timeout:.0f}s energised without reaching target"

        window = float(self.config.stall_window_s.value or 0.0)
        if window > 0 and (now - self._stall_ref_t) >= window:
            moved = height - self._stall_ref_h
            # Progress in the commanded direction (negative if going the wrong way).
            progress = moved if self._moving == "raise" else -moved
            need = float(self.config.stall_min_progress_mm.value or 0.0)
            if progress < need:
                return (
                    f"gate not moving as commanded ({progress:+.1f} mm in "
                    f"{window:.0f}s while {self._moving_word()}) - check hydraulics, "
                    "solenoid wiring and encoder"
                )
            # Progressing normally - slide the window forward.
            self._stall_ref_t = now
            self._stall_ref_h = height
        return None

    async def _drive(self, direction):
        await self._write_outputs(direction == "raise", direction == "lower")

    async def _stop(self, status):
        if self._moving is not None:
            # Mark the moving->idle transition so _begin_move can tell a real
            # rest from an immediate hunting re-engagement.
            self._idle_since = time.monotonic()
            log.info("Stop (%s)", status)
        self._moving = None
        await self._write_outputs(False, False)
        self._status = status

    async def _write_outputs(self, raise_on: bool, lower_on: bool):
        """The ONLY place solenoid/pump outputs are written.

        Enforces the interlocks (never both solenoids energised; never raise
        past the top-limit prox) and writes all pins in a single atomic
        transaction so no intermediate unsafe state can occur. The pump runs
        exactly when a solenoid does - without it the gate cannot move.
        """
        if raise_on and lower_on:  # must never happen - hard guard
            log.error("Interlock violation prevented: raise+lower both requested")
            raise_on = lower_on = False

        if self._top_limit_active and raise_on:
            # Gate is on the over-travel prox: never drive it further up.
            log.warning("Raise blocked: top limit prox active")
            raise_on = False

        pump_on = raise_on or lower_on

        active_low = bool(self.config.do_active_low.value)
        raise_pin = self._raise_pin()
        lower_pin = self._lower_pin()
        pump_pin = self._pump_pin()

        pins: list[int] = []
        values: list[int] = []
        if raise_pin is not None:
            pins.append(raise_pin)
            values.append(self._level(raise_on, active_low))
        if lower_pin is not None:
            pins.append(lower_pin)
            values.append(self._level(lower_on, active_low))
        if pump_pin is not None:
            pins.append(pump_pin)
            values.append(self._level(pump_on, active_low))

        if pins:
            # Native async method; accepts batched pin/value lists so all
            # outputs change in ONE transaction (no unsafe transient).
            await self.platform_iface.set_do(pins, values)

        self._raise_state = raise_on
        self._lower_state = lower_on
        self._pump_state = pump_on

    @staticmethod
    def _level(energised: bool, active_low: bool) -> int:
        return int(energised != active_low)

    def _trip(self, reason):
        if not self._fault:
            log.error("FAULT: %s", reason)
        self._fault = True
        self._fault_reason = reason

    # ------------------------------------------------------------------
    # Top-limit prox: over-travel guard + height calibration
    # ------------------------------------------------------------------
    def _top_limit_level_active(self, level: int) -> bool:
        """Is this DI level 'gate at the top limit'?

        Normally-open prox (the default): active HIGH. Normally-closed prox:
        active LOW, selected with `estop_active_low`.
        """
        active_level = 0 if bool(self.config.estop_active_low.value) else 1
        return int(level) == active_level

    async def _on_top_limit(self, di, di_value, dt_secs, count, edge):
        """Fast path: stop as soon as the prox asserts.

        Reads the pin LEVEL back, never ``di_value``. pydoover's pulse callbacks
        carry ``value`` as the proto3 default False on every driver, real
        hardware included (the encoder app's ``_on_home`` documents the same
        trap), so with a normally-open prox - active HIGH - trusting it would
        mean this fast path never fired at all, silently. One extra read per
        transition, at limit-switch rates, costs nothing.

        Clearing is never done here: only a confirmed inactive level read
        (_poll_top_limit) drops the block, so a single spurious inactive edge
        (noise on the line) can't release the raise-block while the gate is still
        on the sensor.
        """
        try:
            level = await self.platform_iface.fetch_di(di)
        except Exception as e:
            log.warning("Top limit level read failed on DI%s: %s", di, e)
            return
        if level is None:
            # fetch_di soft-fails as None; "unknown" is never "at the limit".
            log.warning("Top limit level unavailable on DI%s", di)
            return
        if self._top_limit_level_active(int(level)):
            await self._engage_top_limit()

    async def _poll_top_limit(self):
        """Level-poll the top limit each loop - the guaranteed backstop.

        The pulse counter can silently drop the lone activating edge (its
        dt_secs>0 filter, the 0.2 s start grace, a ~1 s stream reconnect), so an
        input that blocks an output cannot rely on it. On a failed/None read we
        HOLD the last known state (fail-safe: never release an active block on a
        bad read); a good inactive read is required to clear it.
        """
        limit_pin = self._top_limit_pin()
        if limit_pin is None:
            return
        try:
            level = int(await self.platform_iface.fetch_di(limit_pin))
        except Exception as e:
            log.debug("Top limit level poll failed, holding last state: %s", e)
            return
        active = self._top_limit_level_active(level)
        if active and not self._top_limit_active:
            await self._engage_top_limit()
        elif not active and self._top_limit_active:
            self._top_limit_active = False
            # The next arrival at the prox is a fresh calibration point.
            self._limit_calibrated = False
            log.info("Top limit cleared (gate off the over-travel prox)")

    async def _engage_top_limit(self):
        """Gate reached the top prox: stop now and block raising - no fault.

        A warning, not a trip. Everything is de-energised on arrival because the
        gate was, by definition, on its way up; the next control pass re-engages
        the lower solenoid if the setpoint is below the gate. The re-zero happens
        in main_loop, where the encoder height is available.
        """
        self._top_limit_active = True
        log.warning(
            "Top limit prox active - raising blocked until the gate lowers off it"
        )
        await self._stop("top limit reached - raise blocked")

    def _zero_at_top_limit(self, raw_height: float):
        """Re-zero the gate height against the prox - the calibration datum.

        The prox sits at a known point of travel, so that is what the height is
        anchored to: the offset becomes whatever makes the reading come out at
        `estop_height_mm` (0 by default, i.e. a straight re-zero). Runs once per
        arrival rather than every cycle, so a gate creeping while sitting on the
        sensor can't keep re-zeroing and hide its own movement.
        """
        datum = float(self.config.estop_height_mm.value or 0.0)
        previous = self._height_offset
        self._height_offset = datum - raw_height
        self._limit_calibrated = True
        if self._moving is not None:
            # The calibrated height just stepped. Rebase the stall reference onto
            # the new frame, or the step reads as a jam / a wrong-way move.
            self._stall_ref_t = time.monotonic()
            self._stall_ref_h = datum
        log.info(
            "Height re-zeroed at top limit: encoder %.1f mm now reads %.1f mm "
            "(offset %+.1f mm, was %+.1f mm)",
            raw_height,
            datum,
            self._height_offset,
            previous,
        )

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------
    def _read_raw_height(self):
        """The encoder's height, as published, before calibration is applied."""
        try:
            name = self.config.height_tag_name.value or "Height"
            key = self.config.height_app_key.value
            value = self.get_tag(name, app_key=key)
        except Exception as e:
            log.debug("Height read failed: %s", e)
            return None
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _height_trust_issue(self) -> str | None:
        """Why the (present) height reading can't be trusted yet, else None.

        The encoder's height is incremental: absolute position is only real once
        it has homed, and a frozen encoder keeps its last value in the tag cache
        forever, so both conditions hold the outputs off (non-latching).
        """
        try:
            key = self.config.height_app_key.value
        except Exception:
            return None

        if self.config.require_homed.value and not self.get_tag("Homed", app_key=key):
            return "encoder not homed"

        timeout = float(self.config.heartbeat_timeout_s.value or 0.0)
        if timeout > 0:
            heartbeat = self.get_tag("Heartbeat", app_key=key)
            try:
                age = time.time() - float(heartbeat)
            except (TypeError, ValueError):
                return "no encoder heartbeat"
            if age > timeout:
                return f"encoder signal stale ({age:.0f}s old)"
        return None

    def _read_target(self):
        lo, hi = self.config.height_span
        try:
            target = float(self.ui.target.value)
        except (TypeError, ValueError):
            target = lo
        return max(lo, min(hi, target))

    def _read_mode(self):
        try:
            value = self.ui.mode.value
        except Exception:
            value = None
        return value if value in ("auto", "hold") else "hold"

    def _raise_pin(self):
        value = self.config.raise_do_pin.value
        return int(value) if value is not None else None

    def _lower_pin(self):
        value = self.config.lower_do_pin.value
        return int(value) if value is not None else None

    def _pump_pin(self):
        value = self.config.pump_do_pin.value
        return int(value) if value is not None else None

    def _top_limit_pin(self):
        value = self.config.estop_di_pin.value
        return int(value) if value is not None else None

    def _moving_word(self):
        return {"raise": "raising", "lower": "lowering"}.get(self._moving, "idle")

    # ------------------------------------------------------------------
    # Outputs / publishing
    # ------------------------------------------------------------------
    async def _publish(self, height, target, mode, trust_issue):
        await self.tags.TargetHeight.set(round(target, 2))
        # On signal loss leave GateHeight/Error at their last-known values rather
        # than asserting a false 0 mm ("closed"); HeightValid + the warning flag
        # that the reading is stale.
        if height is not None:
            await self.tags.GateHeight.set(round(height, 2))
            await self.tags.Error.set(round(target - height, 2))
        await self.tags.HeightOffset.set(round(self._height_offset, 2))
        await self.tags.Moving.set(self._moving_word())
        await self.tags.RaiseOutput.set(self._raise_state)
        await self.tags.LowerOutput.set(self._lower_state)
        await self.tags.PumpOutput.set(self._pump_state)
        await self.tags.TopLimitActive.set(self._top_limit_active)
        await self.tags.Mode.set(mode)
        await self.tags.Status.set(self._status)
        await self.tags.Fault.set(self._fault)
        await self.tags.FaultReason.set(self._fault_reason)
        await self.tags.HeightValid.set(height is not None and trust_issue is None)

        self.ui.no_signal_warning.hidden = height is not None
        self.ui.untrusted_warning.hidden = trust_issue is None
        self.ui.top_limit_warning.hidden = not self._top_limit_active
        self.ui.fault_warning.hidden = not self._fault

    # ------------------------------------------------------------------
    # Field actions
    # ------------------------------------------------------------------
    @ui.handler("reset_fault", auto_update=False)
    async def on_reset_fault(self, ctx, value):
        """Clear a latched fault so Auto control can resume."""
        was = self._fault_reason
        self._fault = False
        self._fault_reason = ""
        self._status = "fault reset"
        log.info("Fault reset by operator (was: %s)", was or "none")
        return {"fault": False, "cleared": was}
