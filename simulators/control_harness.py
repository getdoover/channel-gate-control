"""Build a real ``ChannelGateControlApplication`` without the device framework.

Same idea as the encoder repo's harness: stand down ``pydoover``'s
``Application.__init__`` (which wants a device agent, a gRPC platform interface
and a healthcheck port) and leave the app class, its config schema, its tags, its
``setup()`` and its ``main_loop()`` as production code.

The tag layer is the **real** ``pydoover`` ``Tags`` machinery over an in-memory
:class:`FakeTagsManager` keyed by ``app_key``, which is what lets the controller
read the encoder app's ``Height`` tag for real in ``tests/test_interop.py``
instead of through a bespoke double.
"""

from __future__ import annotations

import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

from pydoover.docker import Application

from channel_gate_control.app_config import ChannelGateControlConfig
from channel_gate_control.app_tags import ChannelGateControlTags
from channel_gate_control.application import ChannelGateControlApplication

CONTROL_APP_KEY = "channel_gate_control_1"
ENCODER_APP_KEY = "channel_gate_encoder_1"


class FakeTagsManager:
    """In-memory stand-in for ``TagsManagerDocker``, shared between apps.

    ``Tags``/``BoundTag`` only need ``get_tag`` and ``set_tag`` from a manager.
    Values are stored per ``(app_key, key)`` so cross-app reads behave as they do
    on a device, and every write is timestamped into :attr:`history` so a test can
    assert on publish cadence as well as content.
    """

    def __init__(self):
        self.values: dict[tuple[str | None, str], Any] = {}
        self.history: list[tuple[float, str | None, str, Any]] = []
        self.commits = 0

    def get_tag(
        self,
        key: str,
        default: Any = None,
        app_key: str | None = None,
        raise_key_error: bool = False,
    ) -> Any:
        try:
            return self.values[(app_key, key)]
        except KeyError:
            if raise_key_error:
                raise
            return default

    async def set_tag(
        self,
        key: str,
        value: Any,
        app_key: str | None = None,
        flush: bool = False,
        log: bool = False,
    ) -> None:
        self.values[(app_key, key)] = value
        self.history.append((time.monotonic(), app_key, key, value))

    async def set_tags(
        self, values: dict, app_key: str | None = None, **kwargs
    ) -> None:
        for key, value in values.items():
            await self.set_tag(key, value, app_key=app_key)

    async def commit_tags(self) -> None:
        self.commits += 1

    def subscribe_to_tag(self, key, callback, app_key: str | None = None) -> None:
        """No-op: nothing in these apps depends on tag subscriptions."""

    def writes_of(
        self, key: str, app_key: str | None = None
    ) -> list[tuple[float, Any]]:
        return [(t, v) for t, ak, k, v in self.history if k == key and ak == app_key]


#: A bare ``config.Schema`` does NOT populate defaults -- they are injected from
#: the deployed ``doover_config.json`` -- so reading an unset element raises.
#: Everything the app touches is set explicitly.
CONTROL_DEFAULTS: dict[str, Any] = {
    "height_app_key": ENCODER_APP_KEY,
    "height_tag_name": "Height",
    "require_homed": True,
    "heartbeat_timeout_s": 15.0,
    "height_min_mm": 0.0,
    "height_max_mm": 1000.0,
    "raise_do_pin": 2,
    "lower_do_pin": 3,
    "pump_do_pin": 4,
    "do_active_low": False,
    "deadband_mm": 5.0,
    "hysteresis_mm": 5.0,
    "control_period_s": 0.25,
    "estop_di_pin": None,
    "estop_active_low": False,  # normally-open prox: active HIGH
    "estop_height_mm": 520.0,  # the prox's height: the calibration datum
    "outputs_enabled": True,
    "move_timeout_s": 30.0,
    "stall_window_s": 4.0,
    "stall_min_progress_mm": 3.0,
}


def build_control_config(**overrides: Any) -> ChannelGateControlConfig:
    cfg = ChannelGateControlConfig()
    values = {**CONTROL_DEFAULTS, **overrides}
    for name, value in values.items():
        element = getattr(cfg, name, None)
        if element is not None:
            element.value = value
    return cfg


def _fake_ui(target: float, mode: str) -> SimpleNamespace:
    """Only the UI surface the control app reads and writes.

    ``target`` and ``mode`` are the operator's command inputs; the rest are
    warning indicators the app assigns ``.hidden`` on. Matches the stub style
    already used in ``tests/test_control.py``.
    """
    return SimpleNamespace(
        target=SimpleNamespace(value=target),
        mode=SimpleNamespace(value=mode),
        no_signal_warning=SimpleNamespace(hidden=True),
        untrusted_warning=SimpleNamespace(hidden=True),
        top_limit_warning=SimpleNamespace(hidden=True),
        fault_warning=SimpleNamespace(hidden=True),
    )


def build_control_app(
    platform,
    manager: FakeTagsManager,
    target: float = 0.0,
    mode: str = "hold",
    app_key: str = CONTROL_APP_KEY,
    **config_overrides: Any,
) -> ChannelGateControlApplication:
    """Instantiate the real controller against a simulated platform.

    ``Application.__init__`` is stood down for construction only, so the app's own
    ``__init__`` body still runs and stays the single source of truth for its
    state machine's initial values.
    """
    with patch.object(Application, "__init__", lambda self, *a, **k: None):
        app = ChannelGateControlApplication()

    cfg = build_control_config(**config_overrides)
    app.app_key = app_key
    app.config = cfg
    app.tag_manager = manager
    app.tags = ChannelGateControlTags(app_key, manager, cfg)
    app.ui = _fake_ui(target, mode)
    app.platform_iface = platform
    app.loop_target_period = float(cfg.control_period_s.value)
    return app
