"""
SwarmTech3_UpRising: Backward-Compatible Facade for Core Simulation Engine (sim_core.py)
Preserves legacy import interfaces while delegating to the true 3-Bubble architecture:
  - Bubble 1 (GCS): GCS.backend.c2_gateway (C2Gateway, ROUTE_PRESETS)
  - Bubble 2 (OBC): OBC.backend.drone_firmware (Drone, Tiers), OBC.backend.brains
  - Bubble 3 (SIM): SIM.backend.sitl_engine (SITLWorldEngine, SwarmSimulator, Physics)
"""

from __future__ import annotations
import sys
import os
from pathlib import Path

# Ensure repository root is on sys.path
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# Self-healing Conda Windows DLL path bootstrap:
if sys.platform == "win32":
    _prefix = Path(sys.prefix)
    _dll_candidates = [
        _prefix / "Library" / "bin",
        _prefix / "Scripts",
        _prefix.parent.parent / "Library" / "bin",
    ]
    for _d in _dll_candidates:
        if _d.exists():
            _p_str = str(_d)
            if _p_str not in os.environ.get("PATH", ""):
                os.environ["PATH"] = _p_str + os.pathsep + os.environ.get("PATH", "")
            try:
                os.add_dll_directory(_p_str)
            except (AttributeError, OSError):
                pass

# Re-export from Bubble 2 (OBC)
from OBC.backend.drone_firmware import (
    Drone,
    TIER_LEADER,
    TIER_ESCORT,
    TIER_SCOUT,
)
from OBC.backend.brains import (
    BaseBrain,
    BRAIN_REGISTRY,
    LeaderFollowerBrain,
    GossipConsensusBrain,
)

# Re-export from Bubble 1 (GCS)
from GCS.backend.c2_gateway import (
    C2Gateway,
    ROUTE_PRESETS,
)

# Re-export from Bubble 3 (SIM)
from SIM.backend.sitl_engine import (
    DT,
    BODY_RADIUS,
    ZONE4_CLASH,
    ZONE3_OUTER,
    ZONE2_OUTER,
    V_CRUISE,
    V_MAX,
    V_LKP,
    A_MAX,
    GRAVITY,
    RADIO_RANGE,
    HEARTBEAT_INTERVAL,
    LINK_TIMEOUT,
    TETHER_ANCHOR,
    TETHER_AVOID_DIST,
    TETHER_SNAP_DIST,
    TETHER_MAX_LEN,
    SITLWorldEngine,
    SwarmSimulator,
)

__all__ = [
    "DT",
    "BODY_RADIUS",
    "ZONE4_CLASH",
    "ZONE3_OUTER",
    "ZONE2_OUTER",
    "V_CRUISE",
    "V_MAX",
    "V_LKP",
    "A_MAX",
    "GRAVITY",
    "RADIO_RANGE",
    "HEARTBEAT_INTERVAL",
    "LINK_TIMEOUT",
    "TETHER_ANCHOR",
    "TETHER_AVOID_DIST",
    "TETHER_SNAP_DIST",
    "TETHER_MAX_LEN",
    "TIER_LEADER",
    "TIER_ESCORT",
    "TIER_SCOUT",
    "ROUTE_PRESETS",
    "Drone",
    "BaseBrain",
    "BRAIN_REGISTRY",
    "LeaderFollowerBrain",
    "GossipConsensusBrain",
    "C2Gateway",
    "SITLWorldEngine",
    "SwarmSimulator",
]
