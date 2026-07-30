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

# Fallback for the local switches' press threshold, matching the config schema's
# default. Used when the deployed config carries no readable value - never 0,
# which would read every input level (including 0 V) as "pressed".
_MANUAL_THRESHOLD_FALLBACK_V = 6.0


class ChannelGateControlApplication(Application):
    """Closed-loop height control of a hydraulic channel gate.

    Reads the gate height published by the encoder app, compares it to the
    operator's slider setpoint, and drives one of two solenoid valves to close
    the error. Bang-bang control with a deadband (stop) + hysteresis (re-engage)
    so the valves don't chatter. Two hard safety properties:

      1. The raise and lower solenoids are written through a single choke point
         (`_write_outputs`) as one atomic transaction, so they can never be
         energised together. The pump/motor contactor is written in that same
         transaction, but leads the valve on START by `pump_lead_s` so the pack
         is up to pressure before the valve opens; on STOP everything drops
         together, since dropping the valve late is the unsafe direction.
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

    Local manual control sits on top of all of that: a momentary switch at the
    gate puts 12 V on an analog input, and the gate jogs that way while it is
    held. It works in either mode and, deliberately, ABOVE the fault latch - the
    switch is the on-site recovery path, the one that has to work with a dead
    encoder or a latched stall, and the operator standing at the gate is the
    safety case that replaces the move-timeout and stall detector. Only the
    master output enable outranks it. Holding both switches at once drives
    nothing (resolved before the output choke point), and the top limit still
    hard-blocks a manual raise while leaving manual lower available.
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
        # Pump lead: when the contactor closed for the current drive (None when
        # the pump is off), and whether the valve is still being held off waiting
        # for pressure. Both reset the moment the pump drops out, so every fresh
        # start pays the lead again - the pack has spun down by then.
        self._pump_since: float | None = None
        self._pump_priming: bool = False
        self._top_limit_active: bool = False
        self._top_limit_counter = None  # kept referenced so it isn't garbage-collected
        self._manual_raise_active: bool = False
        self._manual_lower_active: bool = False
        self._manual_counters: list = []  # kept referenced, as above
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

        # Event-driven fast stop. The encoder publishes
        # Height every 0.25 s; reacting in the tag callback cuts the outputs
        # within one publish of reaching the target instead of waiting for the
        # (slower) control loop to come around.
        try:
            self.subscribe_to_tag(
                self.config.height_tag_name.value or "Height",
                self._on_height_tag,
                app_key=self.config.height_app_key.value,
            )
            log.info("Fast-stop height subscription active")
        except Exception as e:
            log.warning("Fast-stop height subscription failed: %s", e)
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

        # A latched fault must survive a restart: the latch exists to stop the
        # gate driving on a broken measurement, and a reboot fixes neither
        # hydraulics nor wiring. Cleared the same way as always - Reset Fault.
        try:
            if self.tags.Fault.value:
                self._fault = True
                self._fault_reason = str(
                    self.tags.FaultReason.value or "fault restored after restart"
                )
                log.warning("Restored latched fault: %s", self._fault_reason)
        except (TypeError, ValueError, AttributeError) as e:
            log.debug("No persisted fault to restore: %s", e)

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

        # Local manual switches. The press is a fast path only: the firmware
        # holds ONE threshold config per VI pin, so arming a VI- release counter
        # on the same pin would overwrite the press config and lose presses
        # entirely. Releases - and any press the stream drops - come from the
        # level poll in main_loop, exactly like the top limit.
        manual_pins = self._manual_pins()
        threshold = self._manual_threshold()
        if manual_pins and threshold <= 0:
            # Nothing is armed and nothing will ever read as pressed. Said once,
            # here, because it is a misconfiguration an operator has to fix - the
            # switch at the gate is silently dead until they do.
            log.warning(
                "Local manual control DISABLED: switch threshold is %g V, and at "
                "or below 0 V every input level would read as pressed",
                threshold,
            )
        else:
            edge = self._manual_edge()
            for direction, pin in manual_pins.items():
                self._manual_counters.append(
                    self.platform_iface.get_new_pulse_counter(
                        pin, edge=edge, callback=self._on_manual_pulse
                    )
                )
                log.info(
                    "Local manual %s switch listener started on AI%s (edge %s)",
                    direction,
                    pin,
                    edge,
                )

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
        # Then the local switches: the loop is where a RELEASE is noticed, so
        # this has to run before the control decision that acts on it.
        await self._poll_manual_inputs()

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
        self._last_calibrated_height = height
        target = self._read_target()
        mode = self._read_mode()
        trust_issue = self._height_trust_issue() if height is not None else None

        await self._control(height, target, mode, trust_issue)
        await self._publish(height, target, mode, trust_issue)

    async def _control(self, height, target, mode, trust_issue):
        # --- Safety / mode gates: any of these hold the outputs off -------
        if not self.config.outputs_enabled.value:
            await self._stop("outputs disabled")
            return

        # Local manual control is checked BEFORE the fault latch, deliberately.
        # The switch at the gate is the on-site recovery path: it has to work in
        # precisely the situations auto control refuses to move in - dead or
        # unhomed encoder, latched stall or move timeout - and the operator
        # physically watching the gate is the safety case that stands in for the
        # automatic protections. It also works in either mode, since Hold only
        # means "don't chase the setpoint". Only the master enable above outranks
        # it. The latch itself is untouched: auto stays off until Reset.
        manual = self._manual_request()
        if manual is not None:
            # The INSTANT a local switch is pressed the
            # mode drops to Hold - the operator taking over must silence auto
            # immediately, not after they let go.
            if not getattr(self, "_manual_hold_pushed", False):
                self._manual_hold_pushed = True
                if self._read_mode() != "hold":
                    log.info("Manual switch pressed - switching mode to Hold")
                    try:
                        await self.ui.mode.set("hold")
                    except Exception as e:
                        log.warning("Failed to push mode to hold: %s", e)
            await self._manual_control(manual)
            return
        self._manual_hold_pushed = False

        if self._fault:
            await self._stop(f"FAULT: {self._fault_reason}")
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
            # Moving: command the stop EARLY (the larger of stop-early and
            # the deadband) so shutoff lag + coast land the gate on target.
            if abs(error) <= max(deadband, self._stop_early_mm()):
                await self._stop("at target")
                return
            # ...or if we've reached/overshot and the error flipped direction.
            want = "raise" if error > 0 else "lower"
            if want != self._moving:
                await self._stop("target reached")
                return
            # Safety checks only apply once a move is underway - and only once
            # the valve is actually open. While the pump is still leading there
            # is nothing to measure, so hold the stall reference at the present
            # height and let its window start from the valve opening rather than
            # from the contactor closing. The move TIMEOUT deliberately keeps
            # running through the lead: a pack that never makes pressure has to
            # trip eventually, and the lead is a fraction of that budget.
            if self._pump_priming:
                self._stall_ref_t = time.monotonic()
                self._stall_ref_h = height
            else:
                reason = self._check_move_safety(height)
                if reason:
                    self._trip(reason)
                    await self._stop(f"FAULT: {reason}")
                    return

        await self._drive(self._moving)
        if self._pump_priming:
            self._status = (
                f"pump lead - {self._moving_word()} in "
                f"{self._priming_remaining_s():.1f}s"
            )

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
        whenever a solenoid does - without it the gate cannot move - but it
        STARTS first, by `pump_lead_s` (see `_apply_pump_lead`).
        """
        if raise_on and lower_on:  # must never happen - hard guard
            log.error("Interlock violation prevented: raise+lower both requested")
            raise_on = lower_on = False

        if self._top_limit_active and raise_on:
            # Gate is on the over-travel prox: never drive it further up.
            log.warning("Raise blocked: top limit prox active")
            raise_on = False

        # Software travel ceiling. At/above the
        # configured max height the gate may only be lowered - blocks manual
        # and auto raising alike (every drive passes through here).
        if raise_on:
            try:
                max_h = float(self.config.height_max_mm.value or 0.0)
                h = getattr(self, "_last_calibrated_height", None)
                if max_h > 0 and h is not None and h >= max_h:
                    log.warning(
                        "Raise blocked: gate at max height (%.1f >= %.1f mm)",
                        h, max_h,
                    )
                    raise_on = False
            except (TypeError, ValueError, AttributeError):
                pass

        pump_on = raise_on or lower_on
        raise_on, lower_on = self._apply_pump_lead(pump_on, raise_on, lower_on)

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

        # The encoder cannot read direction from the
        # DI timing on this hardware, so the commanding side declares it.
        try:
            self._push_direction_hint(raise_on, lower_on)
        except Exception as e:
            log.debug("direction hint push failed: %s", e)

    _HINT_URL = "http://127.0.0.1:8765/direction_hint"
    _HINT_COAST_S = 1.2  # keep the old sign while the wheel decelerates

    def _push_direction_hint(self, raise_on: bool, lower_on: bool):
        """Tell the encoder which way we are commanding the gate.

        raise -> "cw" (count/height increasing), lower -> "ccw", stopped ->
        "none" pushed after a coast window so deceleration edges keep the
        sign of the move that produced them. Failures are logged and ignored:
        the hint is an aid to the encoder, never a gate on control.
        """
        import asyncio
        import json as _json
        import urllib.request

        word = "cw" if raise_on else ("ccw" if lower_on else "none")
        if word == getattr(self, "_hint_word", None):
            return
        self._hint_word = word
        self._hint_gen = getattr(self, "_hint_gen", 0) + 1
        gen = self._hint_gen

        def _post(w):
            # One sendall for the WHOLE request: the encoder's debug server
            # does a single read(), and urllib's separate header/body writes
            # race it - the body is intermittently missed and the hint lands
            # as "none". A single write always arrives in one segment on
            # loopback.
            import socket as _socket

            body = _json.dumps({"direction": w}).encode()
            req = (
                b"POST /direction_hint HTTP/1.1\r\n"
                b"host: 127.0.0.1\r\n"
                b"content-type: application/json\r\n"
                + f"content-length: {len(body)}\r\n".encode()
                + b"connection: close\r\n\r\n"
                + body
            )
            with _socket.create_connection(("127.0.0.1", 8765), timeout=2) as s:
                s.sendall(req)
                resp = s.recv(256)
            if b" 200 " not in resp.split(b"\r\n", 1)[0]:
                raise RuntimeError(f"hint endpoint returned: {resp[:60]!r}")

        loop = asyncio.get_running_loop()

        async def _send():
            if word == "none":
                await asyncio.sleep(self._HINT_COAST_S)
                if self._hint_gen != gen:
                    return  # a new move started during the coast window
            try:
                await loop.run_in_executor(None, _post, word)
                log.info("direction hint -> %s", word)
            except Exception as e:
                log.warning("direction hint POST failed: %s", e)

        asyncio.create_task(_send())

    def _apply_pump_lead(self, pump_on: bool, raise_on: bool, lower_on: bool):
        """Hold the directional valve off until the pump has led by `pump_lead_s`.

        The contactor closes first so the pack is up to pressure, and the motor's
        inrush is over, before the valve opens onto the load. Only the ENERGISE
        edge is staggered: `pump_on` is decided before this runs, so a stop still
        drops valve and pump together in the one transaction - dropping the valve
        late is the unsafe direction.

        Called from `_write_outputs` alone, so every path - auto, the top-limit
        recovery lower, and a manual jog - gets the same lead without knowing
        about it.
        """
        if not pump_on:
            self._pump_since = None
            self._pump_priming = False
            return raise_on, lower_on

        lead = self._pump_lead_s()
        now = time.monotonic()
        if self._pump_since is None:
            self._pump_since = now
            if lead > 0:
                log.info("Pump started - valve held off for %.1fs lead", lead)
        self._pump_priming = (now - self._pump_since) < lead
        if self._pump_priming:
            return False, False
        return raise_on, lower_on

    def _pump_lead_s(self) -> float:
        """Configured pump lead, in seconds.

        Falls back to 0 on an unreadable value - that is exactly the behaviour
        from before this setting existed, so a deployment predating the key
        degrades to "both together" rather than holding the valve shut forever.
        """
        try:
            return max(0.0, float(self.config.pump_lead_s.value or 0.0))
        except (TypeError, ValueError, AttributeError):
            return 0.0

    def _priming_remaining_s(self) -> float:
        """Seconds of pump lead left, for the status line."""
        if self._pump_since is None:
            return 0.0
        return max(0.0, self._pump_lead_s() - (time.monotonic() - self._pump_since))

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
    # Local manual control (momentary switch at the gate)
    # ------------------------------------------------------------------
    def _manual_threshold(self) -> float:
        """Volts at or above which a local switch reads as pressed.

        The one place the threshold is resolved, because getting it wrong is
        fail-DANGEROUS: an unreadable value must never collapse to 0, or every
        reading - 0 V included - counts as a held switch and the gate drives
        itself above the fault latch. So a missing or non-numeric value takes the
        schema default, and a value of 0 or below is read as "local control off"
        (no counters armed, both flags forced released) rather than as a switch
        that is always pressed.
        """
        try:
            return float(self.config.manual_threshold_v.value)
        except (TypeError, ValueError, AttributeError):
            return _MANUAL_THRESHOLD_FALLBACK_V

    def _manual_edge(self) -> str:
        """Firmware edge spec arming a manual switch's press detector.

        The "VI" prefix is what makes the pin argument select an ANALOG input,
        and it arms a sample-to-sample STEP detector rather than a level
        crossing: "VI+6" fires when one poll-to-poll sample jumps UP by more
        than 6 V, i.e. the 0 -> 12 V press. The "@<poll_s>" suffix sets the
        firmware poll rate (0.4 s when omitted), but older platform-interface
        builds crash parsing it - so a configured 0.4 emits the bare legacy form
        that every build understands.
        """
        threshold = self._manual_threshold()
        poll = float(self.config.manual_poll_s.value or 0.4)
        if poll == 0.4:
            return f"VI+{threshold:g}"
        return f"VI+{threshold:g}@{poll:g}"

    def _manual_pins(self) -> dict[str, int]:
        """Configured local switch pins, keyed by the direction each drives."""
        pins: dict[str, int] = {}
        for direction, element in (
            ("raise", self.config.manual_raise_ai_pin),
            ("lower", self.config.manual_lower_ai_pin),
        ):
            value = element.value
            if value is not None:
                pins[direction] = int(value)
        return pins

    def _manual_request(self) -> str | None:
        """What the local switches are asking for right now."""
        if self._manual_raise_active and self._manual_lower_active:
            return "both"
        if self._manual_raise_active:
            return "raise"
        if self._manual_lower_active:
            return "lower"
        return None

    def _set_manual(self, raise_pressed: bool, lower_pressed: bool):
        if raise_pressed != self._manual_raise_active:
            log.info(
                "Local manual RAISE switch %s",
                "pressed" if raise_pressed else "released",
            )
        if lower_pressed != self._manual_lower_active:
            log.info(
                "Local manual LOWER switch %s",
                "pressed" if lower_pressed else "released",
            )
        was_held = self._manual_raise_active or self._manual_lower_active
        self._manual_raise_active = raise_pressed
        self._manual_lower_active = lower_pressed
        if (raise_pressed or lower_pressed) and not was_held:
            self._ensure_release_watcher()

    # Fast release detection. A VI- (falling step)
    # counter can't be armed - the firmware holds ONE threshold config per VI
    # pin and arming release would clobber the press detector - so instead a
    # dedicated task polls the switch LEVELS at the firmware's own 0.1 s VI
    # cadence while a switch is held, and cuts the outputs the moment both
    # read released. Inherits _refresh_manual_levels' fail-safe: an unreadable
    # level counts as released, which now also stops the gate immediately.
    def _ensure_release_watcher(self):
        import asyncio

        task = getattr(self, "_release_watcher", None)
        if task is None or task.done():
            self._release_watcher = asyncio.create_task(self._watch_manual_release())

    async def _watch_manual_release(self):
        import asyncio

        try:
            while self._manual_raise_active or self._manual_lower_active:
                await asyncio.sleep(0.1)
                await self._refresh_manual_levels()
            await self._stop("manual released")
        except Exception as e:
            log.warning("Manual release watcher failed (stopping outputs): %s", e)
            try:
                await self._write_outputs(False, False)
            except Exception:
                pass

    async def _refresh_manual_levels(self):
        """Read the switch inputs and latch pressed/released from the levels.

        The fail-safe direction here is RELEASED - the OPPOSITE of the top
        limit's hold-last-state. The top limit BLOCKS an output, so an unknown
        reading has to keep blocking; these switches DRIVE an output, so an
        unknown reading has to stop driving. An input that moves the gate must
        drop out the moment it can't be read.
        """
        pins = self._manual_pins()
        threshold = self._manual_threshold()
        if not pins or threshold <= 0:
            # Local control isn't configured (or is disabled by a threshold that
            # would read every level as pressed). Force RELEASED rather than just
            # returning: a runtime config update that clears the pins while a
            # switch reads pressed would otherwise leave the flag latched, and it
            # is the flag - not the pin - that drives the gate.
            self._set_manual(False, False)
            return
        directions = list(pins)
        try:
            levels = await self.platform_iface.fetch_ai(*pins.values())
        except Exception as e:
            log.warning("Manual switch read failed, releasing: %s", e)
            self._set_manual(False, False)
            return
        if levels is None:
            # fetch_ai soft-fails as None; "unknown" is never "pressed".
            log.warning("Manual switch levels unavailable, releasing")
            self._set_manual(False, False)
            return
        # One pin returns a bare float, several return a list.
        levels = list(levels) if isinstance(levels, (list, tuple)) else [levels]
        if len(levels) != len(directions):
            # A short (or long) list can't be attributed to pins: zipping it
            # would silently read one switch's level as the other's. Treat it as
            # a failed read and release, like any other unusable answer.
            log.warning(
                "Manual switch read returned %d level(s) for %d pin(s), releasing",
                len(levels),
                len(directions),
            )
            self._set_manual(False, False)
            return

        pressed: dict[str, bool] = {}
        for direction, level in zip(directions, levels):
            try:
                pressed[direction] = float(level) >= threshold
            except (TypeError, ValueError):
                log.warning(
                    "Manual %s switch level unreadable (%r), releasing",
                    direction,
                    level,
                )
                pressed[direction] = False
        self._set_manual(pressed.get("raise", False), pressed.get("lower", False))

    async def _poll_manual_inputs(self):
        """Level-poll the local switches each loop.

        Both the backstop for a press pulse the stream dropped AND the only
        release detector there is: one VI threshold per pin means a pin
        streaming presses cannot also stream releases. Worst-case stop latency
        is therefore one control period - the same as every other stop here.
        """
        await self._refresh_manual_levels()

    async def _manual_control(self, direction):
        """Jog the gate for as long as a local switch is held."""
        if direction == "both":
            # Resolved here, ahead of _write_outputs, so its hard guard - a
            # last-resort assertion that logs an error - never sees an operator
            # action, and so the status says what actually happened.
            await self._stop("manual - both switches pressed")
            return
        if direction == "raise" and self._top_limit_active:
            # _write_outputs would strip the raise anyway; resolving it here
            # keeps the pump from being energised for nothing.
            await self._stop("top limit reached - manual raise blocked")
            return

        if self._moving is not None:
            # An auto move was underway: the operator has taken over. Drop its
            # bookkeeping so the move timers can't carry across - but do NOT mark
            # an idle transition the way _stop does. A manual jog moves the gate
            # an arbitrary distance, so whatever auto does next is a FRESH move,
            # never a hunting continuation of this one; leaving _idle_since set
            # would have _begin_move keep the abandoned move's start time and
            # fault the resumed move on a move timeout it never earned. The
            # outputs are deliberately not written off - we're about to drive them.
            self._moving = None
            self._idle_since = None

        # No _begin_move and no _check_move_safety on purpose: the move timeout
        # and stall detector protect UNATTENDED auto moves, and both need a
        # trusted height - which would fault instantly on the dead encoder that
        # is one of the reasons local control exists. A held deadman switch with
        # the operator watching the gate is its own protection.
        await self._drive(direction)
        word = "manual raise" if direction == "raise" else "manual lower"
        # The lead applies to a jog as well: the operator holding the switch sees
        # the pump run and the gate start a moment later, so say which it is.
        self._status = (
            f"{word} - pump lead {self._priming_remaining_s():.1f}s"
            if self._pump_priming
            else f"{word} (local switch)"
        )

    async def _on_manual_pulse(self, di, di_value, dt_secs, count, edge):
        """Fast path: start jogging on the press step, not a control period later.

        ``di_value`` is never trusted - pydoover delivers the proto3 default
        False on every driver (``_on_top_limit`` documents the same trap) and a
        VI payload carries no analog level at all - so the levels are always
        re-read.

        This only ever STARTS a jog. If the refreshed levels show no request
        (contact bounce, or a switch already let go by the time this runs) it
        does nothing and leaves the loop to decide: stopping belongs to the
        level poll, so the fast path can never fight the auto state machine.
        """
        await self._refresh_manual_levels()
        request = self._manual_request()
        if request is None:
            return
        if not self.config.outputs_enabled.value:
            return
        if request == "raise":
            # The counters are armed in setup(), so a press can land before
            # main_loop has ever polled the prox. Poll it here too, or the
            # never-raise-onto-the-prox invariant would only hold from the first
            # control period rather than from the first pulse.
            await self._poll_top_limit()
        await self._manual_control(request)

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------
    async def _on_height_tag(self, _tag_key, value):
        """Fast-stop: cut outputs the moment an auto move is inside the deadband.

        Runs on every Height tag update from the encoder (0.25 s), so the stop
        happens within one publish of reaching target rather than one control
        loop. Only acts on auto moves this app itself started (_moving set,
        mode auto); manual holds and idle states are untouched. The main loop
        remains the authority for everything else - this only ever STOPS.
        """
        try:
            if self._moving not in ("raise", "lower"):
                return
            if value is None or self._read_mode() != "auto":
                return
            height = float(value) + self._height_offset
            self._last_calibrated_height = height
            target = self._read_target()
            deadband = float(self.config.deadband_mm.value or 0.0)
            if abs(target - height) <= max(deadband, self._stop_early_mm()):
                await self._stop(
                    f"at target ({height:.0f} mm, fast path)"
                )
        except Exception as e:
            log.debug("fast-stop height check failed: %s", e)

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

    async def _push_warning_ui(self, states: dict):
        """Deep-merge warning visibility into the ui_state aggregate.

        Only on change - the aggregate update is a cloud-bound write, so it
        must not run every control period. On failure the cache is left
        stale so the next loop retries.
        """
        if getattr(self, "_last_warning_ui", None) == states:
            return
        try:
            await self.update_channel_aggregate(
                "ui_state",
                {"state": {"children": {self.app_key: {"children": states}}}},
                max_age_secs=-1,
            )
            self._last_warning_ui = states
        except Exception as e:
            log.warning("Failed to push warning visibility to ui_state: %s", e)

    def _stop_early_mm(self) -> float:
        """How many mm before the target the stop is commanded in Auto.

        Valve/pump shutoff lag plus coasting carries the gate past where the
        stop was ISSUED; commanding it early lands the gate ON the target.
        """
        try:
            return float(self.config.stop_early_mm.value)
        except (TypeError, ValueError, AttributeError):
            return 8.0

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
        await self.tags.ManualRaise.set(self._manual_raise_active)
        await self.tags.ManualLower.set(self._manual_lower_active)
        await self.tags.Mode.set(mode)
        await self.tags.Status.set(self._status)
        await self.tags.Fault.set(self._fault)
        await self.tags.FaultReason.set(self._fault_reason)
        await self.tags.HeightValid.set(height is not None and trust_issue is None)

        self.ui.no_signal_warning.hidden = height is not None
        self.ui.untrusted_warning.hidden = trust_issue is None
        self.ui.top_limit_warning.hidden = not self._top_limit_active
        self.ui.fault_warning.hidden = not self._fault
        if self._fault and self._fault_reason:
            fault_text = (
                f"FAULT: {self._fault_reason} - outputs latched off, press Reset"
            )
        else:
            fault_text = "FAULT - outputs latched off, press Reset"
        self.ui.fault_warning.display_name = fault_text

        # The ui_state tree is only published ONCE at setup (values flow as
        # declarative $tag/$cmds references), so element attribute changes like
        # `hidden` never reach the site on their own. Visibility must be pushed
        # into the aggregate explicitly - the site keys BOTH the warning banner
        # and the orange device icon off `hidden` in the reported state.
        await self._push_warning_ui(
            {
                "no_signal": {"hidden": height is not None},
                "height_untrusted": {"hidden": trust_issue is None},
                "top_limit": {"hidden": not self._top_limit_active},
                "fault": {"hidden": not self._fault, "displayString": fault_text},
                # Nested elements must be addressed at their real path in the
                # tree - a bare key here would create a phantom top-level
                # sibling and leave the real element untouched.
                "tabs": {
                    "children": {
                        "control_tab": {
                            "children": {
                                "reset_fault_control": {"hidden": not self._fault},
                            }
                        }
                    }
                },
            }
        )

    # ------------------------------------------------------------------
    # Field actions
    # ------------------------------------------------------------------
    @ui.handler("reset_fault_control", auto_update=False)
    async def on_reset_fault_control(self, ctx, value):
        """Control-tab mirror of Reset Fault (visible only while faulted)."""
        return await self.on_reset_fault(ctx, value)

    @ui.handler("reset_fault", auto_update=False)
    async def on_reset_fault(self, ctx, value):
        """Clear a latched fault so Auto control can resume."""
        was = self._fault_reason
        self._fault = False
        self._fault_reason = ""
        self._status = "fault reset"
        log.info("Fault reset by operator (was: %s)", was or "none")
        return {"fault": False, "cleared": was}
