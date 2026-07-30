from pathlib import Path

from pydoover import ui

from .app_tags import ChannelGateControlTags as T


class ChannelGateControlUI(ui.UI):
    """Cloud HMI for the channel gate controller.

    The operator drags ``target`` to the height they want; ``mode`` switches
    between Auto (seek the target) and Hold (outputs off). Below sit the live
    readouts - actual height, error, the height calibration the top limit prox
    established, which solenoid is driving - then the top-limit warning and the
    fault banner with its reset button. Readouts bind to live tags; the slider,
    mode selector and reset button are command surfaces read/handled by the app.
    """

    # --- Commands ---------------------------------------------------------
    target = ui.Slider(
        "Target Height (mm)",
        min_val=0,
        # Declared span only - setup() replaces both ends with the configured
        # travel limits, so this matches their defaults (0 .. the prox at 520).
        max_val=520,
        step_size=1,
        default=0,
        dual_slider=False,
        inverted=False,
        name="target",
    )
    mode = ui.Select(
        "Control Mode",
        options=[
            ui.Option("Hold"),  # -> value "hold": outputs off
            ui.Option("Auto"),  # -> value "auto": seek target
        ],
        default="hold",
        name="mode",
    )

    # --- Readouts (bound to live tags) ------------------------------------
    height = ui.NumericVariable(
        "Gate Height (mm)",
        precision=1,
        value=ui.bind_tag(T.GateHeight),
        name="height",
    )
    target_readout = ui.NumericVariable(
        "Target (mm)",
        precision=1,
        value=ui.bind_tag(T.TargetHeight),
        name="target_readout",
    )
    error = ui.NumericVariable(
        "Error (mm)",
        precision=1,
        value=ui.bind_tag(T.Error),
        name="error",
    )
    height_offset = ui.NumericVariable(
        "Height Calibration (mm)",
        precision=1,
        value=ui.bind_tag(T.HeightOffset),
        name="height_offset",
    )
    moving = ui.TextVariable(
        "Movement",
        value=ui.bind_tag(T.Moving),
        name="moving",
    )
    status = ui.TextVariable(
        "Status",
        value=ui.bind_tag(T.Status),
        name="status",
    )
    raise_output = ui.BooleanVariable(
        "Raise Solenoid",
        value=ui.bind_tag(T.RaiseOutput),
        name="raise_output",
    )
    lower_output = ui.BooleanVariable(
        "Lower Solenoid",
        value=ui.bind_tag(T.LowerOutput),
        name="lower_output",
    )
    pump_output = ui.BooleanVariable(
        "Pump",
        value=ui.bind_tag(T.PumpOutput),
        name="pump_output",
    )

    # --- Warnings / faults ------------------------------------------------
    no_signal_warning = ui.WarningIndicator(
        "No gate height signal - outputs held",
        name="no_signal",
        hidden=True,
        can_cancel=False,
    )
    untrusted_warning = ui.WarningIndicator(
        "Encoder height not trusted (unhomed/stale) - outputs held",
        name="height_untrusted",
        hidden=True,
        can_cancel=False,
    )
    top_limit_warning = ui.WarningIndicator(
        "TOP LIMIT - raising blocked, lower the gate off the prox",
        name="top_limit",
        hidden=True,
        can_cancel=False,
    )
    fault_warning = ui.WarningIndicator(
        "FAULT - outputs latched off, press Reset",
        name="fault",
        hidden=True,
        can_cancel=False,
    )
    fault_reason = ui.TextVariable(
        "Fault Reason",
        value=ui.bind_tag(T.FaultReason),
        name="fault_reason",
    )
    reset_fault = ui.Button(
        "Reset Fault",
        colour=ui.Colour.red,
        name="reset_fault",
    )

    async def setup(self):
        lo, hi = self.config.height_span
        self.target.min_val = lo
        self.target.max_val = hi
        span = hi - lo
        if span > 0:
            self.height.ranges = [
                ui.Range("Closed", lo, lo + span * 0.05, ui.Colour.blue),
                ui.Range("Part Open", lo + span * 0.05, hi - span * 0.05, ui.Colour.green),
                ui.Range("Open", hi - span * 0.05, hi, ui.Colour.yellow),
            ]


def export():
    ChannelGateControlUI(None, None, None).export(
        Path(__file__).parents[2] / "doover_config.json",
        "channel_gate_control",
    )
