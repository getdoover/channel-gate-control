from pathlib import Path

from pydoover import config


class ChannelGateControlConfig(config.Schema):
    """Config for the channel gate height controller.

    Closed-loop position control of a hydraulically actuated channel gate. The
    operator drags a slider to a target height (mm); the app reads the *actual*
    gate height published by the channel gate encoder app and drives one of two
    solenoid valves (raise / lower) via digital outputs to close the error,
    stopping within a deadband. The two solenoids are hard-interlocked so they
    can never be energised at once.

    Elements are declared as class attributes (pydoover collects these at class
    definition) with an explicit ``name`` so the config key matches the Python
    attribute the app reads.
    """

    # --- Gate height source (the quadrature encoder app) -------------------
    height_app_key = config.ApplicationInstall(
        "Gate Height Source App",
        name="height_app_key",
        description="The channel gate encoder app INSTALL on this device that "
        "publishes the gate height. Its 'Height' tag (mm) is read as the "
        "control feedback.",
    )
    height_tag_name = config.String(
        "Gate Height Tag",
        name="height_tag_name",
        description="Name of the tag carrying gate height in mm on the source "
        "app. The channel gate encoder publishes this as 'Height'.",
        default="Height",
    )
    require_homed = config.Boolean(
        "Require Encoder Homed",
        name="require_homed",
        description="Hold the outputs off until the source app's 'Homed' tag is "
        "true. The encoder's height is incremental and unverified until it has "
        "homed against its limit switch, so driving on an unhomed height can "
        "move the gate to the wrong position. Disable only for height sources "
        "that don't publish a 'Homed' tag.",
        default=True,
    )
    heartbeat_timeout_s = config.Number(
        "Heartbeat Timeout (s)",
        name="heartbeat_timeout_s",
        description="Treat the height as stale (and hold outputs) if the source "
        "app's 'Heartbeat' tag hasn't advanced within this long. Guards against "
        "a dead/frozen encoder while the gate sits at target. 0 disables.",
        default=15.0,
        minimum=0,
    )

    # --- Travel limits (define the slider span) ---------------------------
    height_min_mm = config.Number(
        "Minimum Height (mm)",
        name="height_min_mm",
        description="Gate height at the fully-lowered/closed end of travel. The "
        "target slider cannot be set below this.",
        default=0.0,
    )
    height_max_mm = config.Number(
        "Maximum Height (mm)",
        name="height_max_mm",
        description="Gate height at the fully-raised/open end of travel. The "
        "target slider cannot be set above this.",
        default=1000.0,
    )

    # --- Solenoid outputs -------------------------------------------------
    raise_do_pin = config.Integer(
        "Raise Solenoid DO Pin",
        name="raise_do_pin",
        description="Digital output driving the RAISE (open / lift) hydraulic "
        "solenoid. Leave unset to disable raising.",
        default=None,
        minimum=0,
    )
    lower_do_pin = config.Integer(
        "Lower Solenoid DO Pin",
        name="lower_do_pin",
        description="Digital output driving the LOWER (close / drop) hydraulic "
        "solenoid. Leave unset to disable lowering.",
        default=None,
        minimum=0,
    )
    pump_do_pin = config.Integer(
        "Pump DO Pin",
        name="pump_do_pin",
        description="Digital output driving the hydraulic pump. Energised "
        "whenever either directional solenoid is energised, de-energised "
        "otherwise - without it the gate cannot move. Leave unset if no pump "
        "output is wired.",
        default=None,
        minimum=0,
    )
    do_active_low = config.Boolean(
        "Outputs Active Low",
        name="do_active_low",
        description="Set if a solenoid energises when its DO is driven LOW "
        "(e.g. sinking relay boards). Default: energise on HIGH.",
        default=False,
    )

    # --- Control tuning ---------------------------------------------------
    deadband_mm = config.Number(
        "Deadband (mm)",
        name="deadband_mm",
        description="Stop driving once the gate is within this distance of the "
        "target. Too small hunts around the setpoint; too large parks short.",
        default=5.0,
        minimum=0,
    )
    hysteresis_mm = config.Number(
        "Re-engage Hysteresis (mm)",
        name="hysteresis_mm",
        description="Extra margin beyond the deadband before a *stopped* gate "
        "starts moving again. Stops the solenoids chattering when the gate "
        "drifts slightly off target. Re-engage error = deadband + hysteresis.",
        default=5.0,
        minimum=0,
    )
    control_period_s = config.Number(
        "Control Period (s)",
        name="control_period_s",
        description="How often the control loop evaluates height and drives the "
        "outputs. 0.2-0.5 s suits hydraulic gate control.",
        default=0.25,
        minimum=0.05,
    )

    # --- Safety -----------------------------------------------------------
    estop_di_pin = config.Integer(
        "Top Limit E-Stop DI Pin",
        name="estop_di_pin",
        description="Digital input from the over-travel proximity sensor at the "
        "TOP of the gate. When it triggers, the pump and both solenoids are "
        "de-energised immediately and a fault latches (operator Reset "
        "required). While it stays triggered, raising is hard-blocked but "
        "lowering is allowed so the gate can be recovered off the limit. Leave "
        "unset if not fitted.",
        default=None,
        minimum=0,
    )
    estop_active_low = config.Boolean(
        "E-Stop Active Low",
        name="estop_active_low",
        description="Set for normally-closed proximity sensors that pull the "
        "input LOW when the gate reaches the top limit (the falling edge "
        "triggers the stop). Clear for normally-open sensors.",
        default=True,
    )
    outputs_enabled = config.Boolean(
        "Outputs Enabled",
        name="outputs_enabled",
        description="Master enable. When off, both solenoids are held "
        "de-energised regardless of the setpoint or mode.",
        default=True,
    )
    move_timeout_s = config.Number(
        "Move Timeout (s)",
        name="move_timeout_s",
        description="Maximum time a solenoid may be energised in a single "
        "continuous move before the app faults and stops. Set to the realistic "
        "full-travel time plus margin. 0 disables.",
        default=30.0,
        minimum=0,
    )
    stall_window_s = config.Number(
        "Stall Window (s)",
        name="stall_window_s",
        description="While moving, the gate must make progress within this "
        "window or the app faults (jam, dead encoder, or reversed wiring). "
        "0 disables stall detection.",
        default=4.0,
        minimum=0,
    )
    stall_min_progress_mm = config.Number(
        "Stall Min Progress (mm)",
        name="stall_min_progress_mm",
        description="Minimum height change, in the commanded direction, "
        "expected within each stall window while moving.",
        default=3.0,
        minimum=0,
    )

    @property
    def height_span(self):
        """(low, high) travel limits, ordered low-to-high."""
        lo = float(self.height_min_mm.value or 0.0)
        hi = float(self.height_max_mm.value or 0.0)
        return (lo, hi) if hi >= lo else (hi, lo)


def export():
    ChannelGateControlConfig.export(
        Path(__file__).parents[2] / "doover_config.json", "channel_gate_control"
    )


if __name__ == "__main__":
    export()
