def test_import_application():
    from channel_gate_control.application import ChannelGateControlApplication

    assert ChannelGateControlApplication.config_cls
    assert ChannelGateControlApplication.tags_cls
    assert ChannelGateControlApplication.ui_cls


def test_import_config():
    from channel_gate_control.app_config import ChannelGateControlConfig

    cfg = ChannelGateControlConfig()
    assert cfg.raise_do_pin is not None
    assert cfg.lower_do_pin is not None
    # Travel limits order into (low, high) regardless of how they're entered.
    cfg.height_min_mm.value = 900.0
    cfg.height_max_mm.value = 100.0
    assert cfg.height_span == (100.0, 900.0)


def test_import_ui():
    from channel_gate_control.app_ui import ChannelGateControlUI

    # Framework instantiates the declarative UI with (manager, api, app) = None.
    assert ChannelGateControlUI(None, None, None) is not None


def test_output_level_mapping():
    """Active-high energises on 1; active-low energises on 0."""
    from channel_gate_control.application import ChannelGateControlApplication as A

    assert A._level(True, active_low=False) == 1
    assert A._level(False, active_low=False) == 0
    assert A._level(True, active_low=True) == 0
    assert A._level(False, active_low=True) == 1
