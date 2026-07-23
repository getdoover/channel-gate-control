"""Gate physics simulator for the channel gate controller.

Closes the physical loop on the bench without hardware. It reads the two
solenoid-drive tags the controller publishes (``RaiseOutput`` / ``LowerOutput``)
and integrates a virtual gate height at a fixed actuation speed, then publishes
that height back as the ``Height`` tag the controller reads for feedback.

Because it mirrors a real double-acting cylinder, energising both solenoids at
once produces NO movement (the pressures cancel) - so if the interlock ever
failed, the gate would stick, which is exactly what you'd want to notice.

Tunable via environment (see docker-compose.yml):
  - ``CONTROL_APP_KEY``  app key of the controller to read outputs from
  - ``TRAVEL_MM``        full gate travel (upper clamp), mm
  - ``RATE_MM_S``        actuation speed, mm per second
  - ``START_MM``         starting height, mm
"""

import logging
import os
import time

from pydoover.config import Schema
from pydoover.docker import Application, run_app

log = logging.getLogger()


class GatePhysicsSimulator(Application):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.control_key = os.environ.get("CONTROL_APP_KEY", "test_app_key")
        self.travel_mm = float(os.environ.get("TRAVEL_MM", 1000))
        self.rate_mm_s = float(os.environ.get("RATE_MM_S", 80))
        self.height = float(os.environ.get("START_MM", 400))
        self._last_t = None

    async def setup(self):
        self.loop_target_period = 0.2
        log.info(
            "Gate sim: reading outputs from '%s', %.0f mm travel @ %.0f mm/s, "
            "starting at %.0f mm",
            self.control_key,
            self.travel_mm,
            self.rate_mm_s,
            self.height,
        )
        await self.set_tag("Height", round(self.height, 1))
        # The controller refuses to drive on an unhomed/stale source, so the
        # sim must look like a homed, live encoder.
        await self.set_tag("Homed", True)
        await self.set_tag("Heartbeat", round(time.time(), 1))

    async def main_loop(self):
        now = time.monotonic()
        dt = 0.0 if self._last_t is None else now - self._last_t
        self._last_t = now

        raise_on = bool(self.get_tag("RaiseOutput", app_key=self.control_key))
        lower_on = bool(self.get_tag("LowerOutput", app_key=self.control_key))
        pump_on = bool(self.get_tag("PumpOutput", app_key=self.control_key))

        # No pump -> no oil -> no movement, regardless of the solenoids. This is
        # exactly the failure you'd see on site if the pump output were missed.
        if pump_on and raise_on and not lower_on:
            self.height += self.rate_mm_s * dt
        elif pump_on and lower_on and not raise_on:
            self.height -= self.rate_mm_s * dt
        # pump off, both-on or both-off -> gate holds position

        self.height = max(0.0, min(self.travel_mm, self.height))
        await self.set_tag("Height", round(self.height, 1))
        await self.set_tag("Heartbeat", round(time.time(), 1))


def main():
    c = Schema()
    setattr(c, "_Schema__element_map", {})
    run_app(GatePhysicsSimulator(config=c))


if __name__ == "__main__":
    main()
