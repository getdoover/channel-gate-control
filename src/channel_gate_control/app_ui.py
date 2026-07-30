from pathlib import Path

from pydoover import ui

from .app_tags import ChannelGateControlTags as T

# The two tabs are declared out here, not as class attributes, because the
# declarative metaclass registers every Element in the class body as a TOP-LEVEL
# element - so naming them in the class would render each tab twice, once inside
# the tab bar and once beside it.

# Control: the four things an operator running the gate needs and nothing else -
# where they want it, whether the app is chasing that, where it actually is, and
# the calibration the top limit prox established.
control_tab = ui.Container(
    "Control",
    name="control_tab",
    children=[
        ui.Slider(
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
        ),
        ui.Select(
            "Control Mode",
            options=[
                ui.Option("Hold"),  # -> value "hold": outputs off
                ui.Option("Auto"),  # -> value "auto": seek target
            ],
            default="hold",
            name="mode",
        ),
        ui.NumericVariable(
            "Gate Height (mm)",
            precision=1,
            value=ui.bind_tag(T.GateHeight),
            name="height",
        ),
        ui.NumericVariable(
            "Height Calibration (mm)",
            precision=1,
            value=ui.bind_tag(T.HeightOffset),
            name="height_offset",
        ),
    ],
)

# Diagnostics: the detail you go looking for when the gate isn't doing what you
# asked - what the app thinks it was told, which outputs are live, whether
# someone at the gate is holding a switch, and the fault banner's reason + reset.
diagnostics_tab = ui.Container(
    "Diagnostics",
    name="diagnostics_tab",
    children=[
        ui.NumericVariable(
            "Target (mm)",
            precision=1,
            value=ui.bind_tag(T.TargetHeight),
            name="target_readout",
        ),
        ui.NumericVariable(
            "Error (mm)",
            precision=1,
            value=ui.bind_tag(T.Error),
            name="error",
        ),
        ui.TextVariable(
            "Movement",
            value=ui.bind_tag(T.Moving),
            name="moving",
        ),
        ui.TextVariable(
            "Status",
            value=ui.bind_tag(T.Status),
            name="status",
        ),
        ui.BooleanVariable(
            "Raise Solenoid",
            value=ui.bind_tag(T.RaiseOutput),
            name="raise_output",
        ),
        ui.BooleanVariable(
            "Lower Solenoid",
            value=ui.bind_tag(T.LowerOutput),
            name="lower_output",
        ),
        ui.BooleanVariable(
            "Pump",
            value=ui.bind_tag(T.PumpOutput),
            name="pump_output",
        ),
        ui.BooleanVariable(
            "Manual Up Switch",
            value=ui.bind_tag(T.ManualRaise),
            name="manual_raise",
        ),
        ui.BooleanVariable(
            "Manual Down Switch",
            value=ui.bind_tag(T.ManualLower),
            name="manual_lower",
        ),
        ui.TextVariable(
            "Fault Reason",
            value=ui.bind_tag(T.FaultReason),
            name="fault_reason",
        ),
        ui.Button(
            "Reset Fault",
            colour=ui.Colour.red,
            name="reset_fault",
        ),
    ],
)


class ChannelGateControlUI(ui.UI):
    """Cloud HMI for the channel gate controller, in two tabs.

    **Control** is the running surface: drag ``target`` to the height you want,
    switch ``mode`` between Auto (seek the target) and Hold (outputs off), and
    watch the actual height and the calibration the top limit prox established.

    **Diagnostics** is everything you only look at when the gate is misbehaving -
    the target as the app read it, the error, which solenoid and pump are live,
    whether someone at the gate is holding a local switch, and the fault reason
    with its Reset button.

    The four warning indicators stay OUTSIDE the tabs, at the top level, so a
    lost height signal, an untrusted reading, the top limit or a latched fault is
    visible whichever tab happens to be open.

    Readouts bind to live tags; the slider, mode selector and reset button are
    command surfaces read/handled by the app.
    """

    tabs = ui.TabContainer(
        "Gate Control",
        name="tabs",
        children=[control_tab, diagnostics_tab],
    )

    # --- Warnings / faults (top level: visible from either tab) -----------
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

    # --- Flat accessors for the elements that moved into a tab ------------
    # The app reads `self.ui.target.value` / `self.ui.mode.value`, and setup()
    # below rewrites the slider's span and the height gauge's ranges. Tabbing
    # pushed those elements a level down, so these keep the flat names working
    # rather than scattering `tabs.control_tab.` through the call sites.
    #
    # Safe per instance: `Container.add_children` setattr's each child onto its
    # container, and `ui.UI.__init__` deep-copies the whole declared template, so
    # the nested lookup lands on THIS instance's copies. Properties aren't
    # Element instances, so the declarative metaclass ignores them - they never
    # become elements in their own right.
    @property
    def target(self):
        return self.tabs.control_tab.target

    @property
    def mode(self):
        return self.tabs.control_tab.mode

    @property
    def height(self):
        return self.tabs.control_tab.height

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
