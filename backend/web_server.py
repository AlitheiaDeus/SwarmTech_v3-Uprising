"""
SwarmTech3_UpRising: Web & WebSocket Telemetry Server (Backward-Compatible Facade)
Re-exports from GCS.backend.web_server
"""

from __future__ import annotations
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from GCS.backend.web_server import SwarmWebServer

__all__ = ["SwarmWebServer"]
