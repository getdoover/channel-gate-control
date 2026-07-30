"""In-process hydraulic gate rig: closes the physical loop between the two apps.

``simulators/sample/main.py`` is the docker version of this -- it reads the
controller's ``RaiseOutput``/``LowerOutput`` tags and integrates a virtual height.
This is the same physics with two differences that matter for testing:

1. It reads the **actual digital outputs** the controller wrote, not the tags
   mirroring them, so the solenoid interlock and the pump gating are exercised for
   real rather than trusted.
2. It does not publish ``Height`` itself. It converts the height it integrated
   into **quadrature pulses on the encoder's two DI pins**, and lets the real
   encoder app decode them and publish the height. That is what makes an
   end-to-end test end-to-end: solenoid -> oil -> gate -> toothed target -> two
   prox sensors -> rising-edge callbacks -> position -> tag -> controller ->
   solenoid.

Like a real double-acting cylinder, energising both solenoids at once produces no
movement (the pressures cancel), and no pump means no oil means no movement -- so
an interlock failure shows up as a gate that sticks.
"""

from __future__ import annotations

import asyncio
import time


class HydraulicGateRig:
    """Integrates gate height from the controller's outputs and emits pulses.

    Parameters
    ----------
    platform
        The shared platform simulator both apps are bound to. Digital outputs are
        read back from it; the encoder's digital inputs are driven through
        ``gate``.
    gate
        A quadrature pulse injector (the encoder repo's ``QuadratureGateSim``).
        Only ``.direction``, ``.step()`` and ``.true_position`` are used, so this
        module needs no import from the other repo.
    rate_mm_s
        Actuation speed. Only RISING edges are captured, so one decoded count is
        half a tooth cycle (``mm_per_count``) and one ``gate.step()`` is a QUARTER
        cycle. The rate that matters to the encoder is therefore
        ``rate_mm_s / (2 * mm_per_count)`` rising edges/s per sensor: the default
        60 mm/s with ``mm_per_count=2.0`` gives the required 15.
    top_limit_pin, top_limit_mm
        Optional over-travel prox: the DI is driven active once the gate reaches
        ``top_limit_mm``, and released when it comes back below. Nothing else in
        the rig knows about it, so the controller has to notice through the pin -
        which is the point.
    """

    def __init__(
        self,
        platform,
        gate,
        raise_pin: int = 2,
        lower_pin: int = 3,
        pump_pin: int = 4,
        rate_mm_s: float = 60.0,
        mm_per_count: float = 2.0,
        travel_mm: float = 1000.0,
        start_mm: float = 0.0,
        top_limit_pin: int | None = None,
        top_limit_mm: float | None = None,
        top_limit_active_low: bool = False,
    ):
        self.platform = platform
        self.gate = gate
        self.raise_pin = raise_pin
        self.lower_pin = lower_pin
        self.pump_pin = pump_pin
        self.rate_mm_s = float(rate_mm_s)
        self.mm_per_count = float(mm_per_count)
        self.travel_mm = float(travel_mm)
        self.top_limit_pin = top_limit_pin
        self.top_limit_mm = None if top_limit_mm is None else float(top_limit_mm)
        self.top_limit_active_low = bool(top_limit_active_low)
        #: None until the first tick, so the pin's resting level gets established
        #: even when the gate starts well clear of the sensor.
        self._top_limit_level: bool | None = None
        #: Ground-truth gate height. The encoder's published height is a
        #: measurement OF this, and the gap between them is the thing under test.
        self.height = float(start_mm)
        self.peak_height = float(start_mm)
        self.trough_height = float(start_mm)
        #: Seconds either solenoid was energised -- the hydraulic duty.
        self.drive_time_s = 0.0
        self.reversals = 0
        self._last_direction = 0
        #: Quarter cycles of the target waveform emitted so far. Quarter, not
        #: half: only two of the four edges in a cycle are rising, so the waveform
        #: has to be walked at quarter resolution for the rising edges to land in
        #: the right places and with the right 90 degree spacing.
        #:
        #: Seeded from the starting height, because the toothed target's phase at
        #: boot is simply wherever the gate happens to be - it did not travel
        #: there. The encoder's count is independent of it (it counts from
        #: wherever it homed), which is exactly how a gate can sit at a real
        #: height with the encoder reading something else.
        self._emitted_quarters = int(self.height / (self.mm_per_count / 2.0))

    def _energised(self) -> tuple[bool, bool, bool]:
        do = self.platform.do_levels
        return (
            bool(do.get(self.raise_pin, False)),
            bool(do.get(self.lower_pin, False)),
            bool(do.get(self.pump_pin, False)),
        )

    async def tick(self, dt: float):
        """Advance the physics by ``dt`` and emit any pulses that implies."""
        raise_on, lower_on, pump_on = self._energised()

        direction = 0
        if pump_on and raise_on and not lower_on:
            direction = 1
        elif pump_on and lower_on and not raise_on:
            direction = -1

        if direction:
            self.height += direction * self.rate_mm_s * dt
            self.drive_time_s += dt
            if self._last_direction and direction != self._last_direction:
                self.reversals += 1
            self._last_direction = direction
        else:
            self._last_direction = 0

        self.height = max(0.0, min(self.travel_mm, self.height))
        self.peak_height = max(self.peak_height, self.height)
        self.trough_height = min(self.trough_height, self.height)
        await self._drive_top_limit()
        await self._emit_pulses()

    async def _drive_top_limit(self):
        """Assert/release the over-travel prox DI from the true gate height."""
        if self.top_limit_pin is None or self.top_limit_mm is None:
            return
        active = self.height >= self.top_limit_mm
        level = (not active) if self.top_limit_active_low else active
        if level != self._top_limit_level:
            self._top_limit_level = level
            await self.platform.set_di_level(self.top_limit_pin, level)

    async def _emit_pulses(self):
        """Walk the quadrature output until it represents the current height.

        One decoded count (one rising edge) per ``mm_per_count`` of travel, so a
        quarter cycle per ``mm_per_count / 2``, emitted as a proper Gray-code walk
        so the encoder has to decode real direction from real rise timing rather
        than being handed a number.
        """
        mm_per_quarter = self.mm_per_count / 2.0
        target = int(self.height / mm_per_quarter)
        while self._emitted_quarters != target:
            step = 1 if target > self._emitted_quarters else -1
            self.gate.direction = step
            await self.gate.step()
            self._emitted_quarters += step

    async def run_until(
        self,
        predicate,
        timeout_s: float,
        dt: float = 1.0 / 60.0,
    ) -> bool:
        """Run the physics until ``predicate()`` is true or ``timeout_s`` elapses.

        Returns whether the predicate was satisfied. ``dt`` is the physics step;
        60 Hz is comfortably finer than either app's control period so the apps,
        not the integrator, set the pace.
        """
        start = time.monotonic()
        while (time.monotonic() - start) < timeout_s:
            await self.tick(dt)
            await asyncio.sleep(dt)
            if predicate():
                return True
        return False
