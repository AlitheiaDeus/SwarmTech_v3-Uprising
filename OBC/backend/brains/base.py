"""
SwarmTech3_UpRising: Base Guidance Brain Protocol (base.py)
Abstract base interface for swappable guidance and comms plugins.
"""

from __future__ import annotations
import math
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

    def compute_gnss_denied_guidance(self, drone: Drone, dt: float) -> np.ndarray:
        """Compute autonomous dead-reckoning RTB guidance for GNSS-denied airframe (TKT-52).
        STRICT AIRGAP & NON-CHEATING INVARIANT:
        The OBC is strictly forbidden from reading true spatial coordinates (drone.pos[:2] or
        sim.target_pos) from simulation memory. Guidance relies exclusively on:
        1. Compass Heading (drone.reciprocal_heading)
        2. Barometric Altitude (drone.pos[2])
        3. Inertial Velocity Damping (drone.vel)
        4. GCS Mast Howl RF Beacon Gradient (drone.rssi vs drone.rssi_prev)
        """
        drone.gnss_denied_timer += dt

        # 1. Time-Scaled Drift Uncertainty: R_drift(dt) = min(15.0m, 0.5m + 0.4 * dt)
        drone.drift_radius = min(15.0, 0.5 + 0.4 * drone.gnss_denied_timer)

        # 2. Check Emergency Blind Soft-Settle criteria
        # Near-field beacon threshold (RSSI >= -45.0 dBm) or 25.0s timeout
        if drone.rssi >= -45.0 or drone.gnss_denied_timer >= 25.0:
            drone.gnss_settled = True

        if getattr(drone, "gnss_settled", False):
            # Phase B: Emergency Blind Soft-Settle
            # Zero horizontal velocity
            ax = float(np.clip(-4.0 * drone.vel[0], -6.0, 6.0))
            ay = float(np.clip(-4.0 * drone.vel[1], -6.0, 6.0))
            # Smooth -0.8 m/s vertical descent
            target_vz = -0.8
            az = float(np.clip(3.5 * (target_vz - drone.vel[2]), -5.0, 5.0))
            return np.array([ax, ay, az], dtype=float)

        # Phase A: Reciprocal Heading Dead-Reckoning Transit & Vertical Altitude Staircase
        # 1. Vertical Altitude Staircase: Z_k = max(6.0m, 25.0m - (k * 2.0m))
        target_z = max(6.0, 25.0 - (drone.id * 2.0))
        az = float(np.clip(3.0 * (target_z - drone.pos[2]) - 2.5 * drone.vel[2], -6.0, 6.0))

        # 2. RF Beacon Gradient Tracking (Hot/Cold)
        # Delta RSSI = RSSI(t) - RSSI(t - dt)
        if getattr(drone, "rssi_prev", -100.0) > -99.0:
            delta_rssi = drone.rssi - drone.rssi_prev
            if delta_rssi < -0.05:
                # Fading signal: apply a small search trim to seek the signal peak
                trim = 0.05 if (drone.id % 2 == 0) else -0.05
                drone.reciprocal_heading = float((drone.reciprocal_heading + trim) % (2.0 * math.pi))
        drone.rssi_prev = drone.rssi

        # 3. Reciprocal Heading Vector & Lateral Fan-Out
        psi = drone.reciprocal_heading
        u_parallel = np.array([math.cos(psi), math.sin(psi)], dtype=float)
        u_perp = np.array([-math.sin(psi), math.cos(psi)], dtype=float)

        # Lateral fan-out (even Starboard +2.0m/s, odd Port -2.0m/s scaled by drift confidence)
        side = 1.0 if (drone.id % 2 == 0) else -1.0
        v_lateral = side * 2.0 * min(1.0, drone.drift_radius / 5.0)
        v_cruise = 8.0  # Conservative dead-reckoning cruise speed

        v_target_xy = v_cruise * u_parallel + v_lateral * u_perp
        ax = float(np.clip(3.0 * (v_target_xy[0] - drone.vel[0]), -6.0, 6.0))
        ay = float(np.clip(3.0 * (v_target_xy[1] - drone.vel[1]), -6.0, 6.0))

        return np.array([ax, ay, az], dtype=float)
