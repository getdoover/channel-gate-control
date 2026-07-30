"""Make the sibling channel-gate-encoder repo importable, when it is present.

``test_interop.py`` runs the REAL encoder app and the REAL controller in one
event loop, so it needs the encoder repo's ``src/`` and ``simulators/`` on the
path. That repo is a separate deployable, so it is not (and should not become) a
dependency in ``pyproject.toml``: adding a local path dependency would break CI,
where only this repo is checked out.

Instead the path is added when the sibling checkout exists. When it does not,
:data:`ENCODER_REPO` is ``None`` and the interop tests skip with a clear reason.

Set ``GATE_ENCODER_REPO`` to point at the checkout explicitly, or at a path that
does not exist to force the interop tests to skip.
"""

import os
import sys
from pathlib import Path

#: Resolved path to the encoder repo, or None if it isn't available.
ENCODER_REPO: Path | None = None

_candidate = Path(
    os.environ.get(
        "GATE_ENCODER_REPO",
        Path(__file__).resolve().parents[2] / "channel-gate-encoder",
    )
)
if (_candidate / "src" / "channel_gate_encoder").is_dir():
    ENCODER_REPO = _candidate
    # Both subdirectories are needed: src/ for the app package, simulators/ for
    # the platform simulator and the encoder's app harness.
    for sub in ("src", "simulators"):
        path = str(_candidate / sub)
        if path not in sys.path:
            sys.path.insert(0, path)
