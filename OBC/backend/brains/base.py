"""
SwarmTech3_UpRising: Base Guidance Brain Protocol (base.py)
Abstract base interface for swappable guidance and comms plugins.
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Any, TYPE_CHECKING
import numpy as np

if TYPE_CHECKING:
    from SIM.backend.sitl_engine import SwarmSimulator
    from OBC.backend.drone_firmware import Drone


class BaseBrain(ABC):
    """Abstract interface for autonomous swarm guidance brains.
    Separates mission planning, peer communications, and target allocation
    from the underlying 60 FPS physics integration and collision safety hulls."""

    brain_name: str = "BASE"

    def __init__(self, sim: SwarmSimulator):
        self.sim: SwarmSimulator = sim

    @abstractmethod
    def reset(self) -> None:
        """Reset internal brain states upon fleet re-initialization."""
        pass

    @abstractmethod
    def update_comms(self, dt: float) -> None:
        """Simulate realistic radio packet exchanges, telemetry heartbeats,
        and link timeout evaluations."""
        pass

    @abstractmethod
    def compute_guidance(self, dt: float) -> Dict[int, np.ndarray]:
        """Compute the mission guidance steering vector for each active drone.
        Returns a dictionary mapping drone.id -> desired acceleration (m/s^2)."""
        pass

    def handle_command(self, cmd_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Optional hook to handle brain-specific commands (e.g. echo injection,
        formation mode switch, succession triggers). Returns response dict or None."""
        return None

    def on_succession(self, old_leader: Optional[Drone], new_leader: Drone) -> None:
        """Optional hook invoked when autonomic succession elects a new leader."""
        pass

    def get_telemetry_extra(self) -> Dict[str, Any]:
        """Optional dictionary of brain-specific state variables to include
        in the WebSocket JSON telemetry stream."""
        return {}
