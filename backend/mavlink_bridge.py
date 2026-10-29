"""
SwarmTech3_UpRising: UDP & MAVLink Network Bridge (Backward-Compatible Facade)
Re-exports from SIM.backend.mavlink_bridge
"""

from __future__ import annotations
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from SIM.backend.mavlink_bridge import (
    MavlinkBridge,
    HAVE_PYMAVLINK,
    DEFAULT_HOST,
    DEFAULT_TX_PORT,
    DEFAULT_RX_PORT,
    DEFAULT_MAVLINK_PORT,
    HOME_LAT,
    HOME_LON,
    HOME_ALT,
)

__all__ = [
    "MavlinkBridge",
    "HAVE_PYMAVLINK",
    "DEFAULT_HOST",
    "DEFAULT_TX_PORT",
    "DEFAULT_RX_PORT",
    "DEFAULT_MAVLINK_PORT",
    "HOME_LAT",
    "HOME_LON",
    "HOME_ALT",
]
