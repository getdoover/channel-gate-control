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
        self._estop_active: bool = False
        self._estop_counter = None  # kept referenced so it isn't garbage-collected
        self._idle_since: float | None = None  # when we last stopped (episode gap)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    async def setup(self):
        self.loop_target_period = float(self.config.control_period_s.value or 0.25)
        # Establish a known-safe output state before any control runs. Important
        # for active-low wiring, where an undriven pin reads as energised.
        await self._write_outputs(False, False)

        # Top-limit e-stop. A hard safety limit must not depend on a single
        # edge being delivered, so detection is LEVEL-driven: main_loop polls
        # the DI every cycle (_poll_estop) as the guaranteed backstop. The
        # both-edge pulse counter is only the fast path for an immediate trip.
        # No initial read is needed here - the first main_loop poll runs before
        # any output is driven, so a gate already on the limit at boot trips
        # before it can be raised.
        estop_pin = self._estop_pin()
        if estop_pin is not None:
            try:
                await self.platform_iface.set_di_config(estop_pin, debounce_ms=20)
            except Exception as e:
                log.debug("set_di_config(estop %s) failed: %s", estop_pin, e)
            self._estop_counter = self.platform_iface.get_new_pulse_counter(
                estop_pin, edge="both", callback=self._on_estop
            )
            log.info("Top limit e-stop listener started on DI%s", estop_pin)

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
        # Poll the safety limit FIRST, before any control decision or drive, so
        # a missed activating edge is caught within one control period.
        await self._poll_estop()

        height = self._read_height()
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

        if self._estop_active:
            # Top limit still asserted (e.g. after an operator reset while the
            # gate is still on the sensor). Raising is hard-blocked; the only
            # recovery is to lower AWAY from the over-travel limit. Stall/timeout
            # detection is deliberately skipped here - with raise suppressed it
            # would otherwise refault with a misleading 'gate not moving' reason.
            if error < -deadband:
                self._moving = "lower"
                await self._drive("lower")
                self._status = "top limit e-stop - lowering to recover"
            else:
                await self._stop("top limit e-stop - lower to recover")
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
        past the top-limit e-stop) and writes all pins in a single atomic
        transaction so no intermediate unsafe state can occur. The pump runs
        exactly when a solenoid does - without it the gate cannot move.
        """
        if raise_on and lower_on:  # must never happen - hard guard
            log.error("Interlock violation prevented: raise+lower both requested")
            raise_on = lower_on = False

        if self._estop_active and raise_on:
            # Gate is on the over-travel sensor: never drive it further up.
            log.warning("Raise blocked: top limit e-stop active")
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
    # Top-limit e-stop
    # ------------------------------------------------------------------
    def _estop_level_active(self, level: int) -> bool:
        active_level = 0 if bool(self.config.estop_active_low.value) else 1
        return int(level) == active_level

    async def _on_estop(self, di, di_value, dt_secs, count, edge):
        # Fast path only: trip immediately on an ACTIVATING edge. Clearing is
        # never done here - only a confirmed inactive level read (_poll_estop)
        # drops the block, so a single spurious inactive edge (noise on the NC
        # line) can't release the raise-block while the gate is still on the
        # sensor.
        if self._estop_level_active(int(di_value)):
            await self._estop_trip()

    async def _poll_estop(self):
        """Level-poll the top-limit e-stop each loop - the guaranteed backstop.

        The pulse counter can silently drop the lone activating edge (its
        dt_secs>0 filter, the 0.2 s start grace, a ~1 s stream reconnect), so a
        hard safety limit cannot rely on it. On a failed/None read we HOLD the
        last known state (fail-safe: never release an active block on a bad
        read); a good inactive read is required to clear it.
        """
        estop_pin = self._estop_pin()
        if estop_pin is None:
            return
        try:
            active = self._estop_level_active(int(await self.platform_iface.fetch_di(estop_pin)))
        except Exception as e:
            log.debug("e-stop level poll failed, holding last state: %s", e)
            return
        if active and not self._estop_active:
            await self._estop_trip()
        elif not active and self._estop_active:
            # Confirmed clear. The latched fault still requires an operator
            # Reset; this only lifts the raise-block once the gate is off the limit.
            self._estop_active = False
            log.info("Top limit e-stop cleared (gate off the over-travel sensor)")

    async def _estop_trip(self):
        """Immediate stop: pump + both solenoids off NOW, fault latched."""
        self._estop_active = True
        self._trip("top limit e-stop - gate at over-travel sensor")
        await self._stop("FAULT: top limit e-stop")

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------
    def _read_height(self):
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

    def _estop_pin(self):
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
        await self.tags.Moving.set(self._moving_word())
        await self.tags.RaiseOutput.set(self._raise_state)
        await self.tags.LowerOutput.set(self._lower_state)
        await self.tags.PumpOutput.set(self._pump_state)
        await self.tags.EStopActive.set(self._estop_active)
        await self.tags.Mode.set(mode)
        await self.tags.Status.set(self._status)
        await self.tags.Fault.set(self._fault)
        await self.tags.FaultReason.set(self._fault_reason)
        await self.tags.HeightValid.set(height is not None and trust_issue is None)

        self.ui.no_signal_warning.hidden = height is not None
        self.ui.untrusted_warning.hidden = trust_issue is None
        self.ui.estop_warning.hidden = not self._estop_active
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
