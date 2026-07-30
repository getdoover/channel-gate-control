"""Tests for the cloud UI's shape - the Control / Diagnostics tab split.

Tabbing the flat element list pushes the slider, the mode selector and the Reset
button a level down the tree, and two things that have to survive that are not
obvious from reading the declaration:

  * ``get_interactions()`` still has to find the nested command elements - that
    dict is what routes an operator's command to ``@ui.handler`` - and
  * the app reads ``self.ui.target`` / ``self.ui.mode`` flat, so the accessors
    standing in for the moved elements must resolve to THIS instance's copies,
    not to the shared class template.
"""

from control_harness import build_control_config

from channel_gate_control.app_ui import ChannelGateControlUI


def _ui():
    """A UI instance as the framework builds one: (config, tags, app_key) = None."""
    return ChannelGateControlUI(None, None, None)


def test_commands_are_still_reachable_as_interactions():
    interactions = _ui().get_interactions()
    # The collector recurses through `_children`, so two containers deep is fine.
    for name in ("target", "mode", "reset_fault"):
        assert name in interactions
    # The warnings are interactions too, and they stayed at the top level.
    for name in ("no_signal", "height_untrusted", "top_limit", "fault"):
        assert name in interactions


def test_schema_nests_the_elements_under_the_tabs():
    schema = _ui().to_schema(resolve_config=False)
    assert schema["children"]["tabs"]["type"] == "uiTabs"

    tabs = schema["children"]["tabs"]["children"]
    assert list(tabs) == ["control_tab", "diagnostics_tab"]
    assert list(tabs["control_tab"]["children"]) == [
        "target",
        "mode",
        "reset_fault_control",
        "height",
        "height_offset",
    ]
    assert list(tabs["diagnostics_tab"]["children"]) == [
        "target_readout",
        "error",
        "moving",
        "status",
        "raise_output",
        "lower_output",
        "pump_output",
        "manual_raise",
        "manual_lower",
        "fault_reason",
        "reset_fault",
    ]

    # Warnings stay OUTSIDE the tabs, so a lost signal or a latched fault shows
    # whichever tab is open. Control is declared first, which makes it the
    # default page - no explicit defaultPage to keep in step.
    assert list(schema["children"]) == [
        "tabs",
        "no_signal",
        "height_untrusted",
        "top_limit",
        "fault",
    ]
    assert "defaultPage" not in schema["children"]["tabs"]


def test_flat_accessors_reach_this_instance_and_not_the_template():
    one, two = _ui(), _ui()
    assert one.target is one.tabs.control_tab.target
    assert one.target is not two.target

    one.target.max_val = 999
    assert two.target.max_val != 999


async def test_setup_applies_the_configured_travel_limits():
    """setup() mutates the now-nested slider and height gauge in place."""
    instance = _ui()
    instance.config = build_control_config(height_min_mm=100.0, height_max_mm=600.0)
    await instance.setup()

    assert (instance.target.min_val, instance.target.max_val) == (100.0, 600.0)
    # Same object either way round - the accessor is not a copy.
    assert instance.tabs.control_tab.target.max_val == 600.0
    assert [r.label for r in instance.height.ranges] == ["Closed", "Part Open", "Open"]
