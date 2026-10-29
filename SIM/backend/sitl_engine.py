"""
SwarmTech3_UpRising: Core Simulation Engine (sim_core.py)
Vectorized 60 FPS Boid Kinematics, Sane 4-Zone Spatial Safety,
Tether Collision & Snap Dynamics, 10 Hz Baud Radio Comms & LKP Recovery,
and Autonomic Leadership Succession.
"""

from __future__ import annotations
import math
import time
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any, Set

# Ensure repository root is on sys.path
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
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

import numpy as np

from OBC.backend.drone_firmware import Drone, TIER_LEADER, TIER_ESCORT, TIER_SCOUT
from OBC.backend.brains import BaseBrain, BRAIN_REGISTRY, LeaderFollowerBrain, GossipConsensusBrain
from GCS.backend.c2_gateway import C2Gateway, ROUTE_PRESETS


# ==============================================================================
# Physical Constants & Tuning Parameters
# ==============================================================================
DT: float = 1.0 / 60.0                 # 60 FPS simulation time step (s)
BODY_RADIUS: float = 0.75              # Physical drone airframe radius (m) -> 1.5m wingspan
ZONE4_CLASH: float = 1.5               # Prop clash collision threshold (m)
ZONE3_OUTER: float = 3.5               # Critical evasion & velocity-damping zone (m)
ZONE2_OUTER: float = 8.0               # Smooth separation caution zone (m)

V_CRUISE: float = 12.0                 # Standard cruise speed (m/s)
V_MAX: float = 20.0                    # Hard velocity clamp (m/s)
V_LKP: float = 8.0                     # Conservative speed when navigating to LKP (m/s)
A_MAX: float = 15.0                    # Maximum acceleration (m/s^2)
GRAVITY: float = -9.81                 # Ballistic drop acceleration (m/s^2)

# Radio Comms & LKP
RADIO_RANGE: float = 120.0             # Radio broadcast range limit (m)
HEARTBEAT_INTERVAL: float = 0.1        # 10 Hz broadcast interval (s)
LINK_TIMEOUT: float = 1.0              # Lost-link timeout threshold (s)

# Tether Dynamics
TETHER_ANCHOR = np.array([0.0, 0.0, 1.5], dtype=float)  # GCS mast anchor (m)
TETHER_AVOID_DIST: float = 3.0         # Follower clearance envelope from cable (m)
TETHER_SNAP_DIST: float = 1.0          # Follower collision severance threshold (m)
TETHER_MAX_LEN: float = 300.0          # Max cable length before tensile snap (m)

# Hardware Tiers for Succession Scoring
TIER_LEADER: int = 3
TIER_ESCORT: int = 2
TIER_SCOUT: int = 1

# Discrete Waypoint Mission Route Presets (TKT-28)
ROUTE_PRESETS: Dict[str, List[List[float]]] = {
    "BOX_PATROL": [
        [40.0, -20.0, 25.0],
        [40.0, 40.0, 28.0],
        [-20.0, 40.0, 25.0],
        [-20.0, -20.0, 22.0],
    ],
    "DIAMOND_SURVEY": [
        [50.0, 10.0, 28.0],
        [15.0, 45.0, 25.0],
        [-20.0, 10.0, 24.0],
        [15.0, -25.0, 26.0],
    ],
    "HEXAGON_SWEEP": [
        [45.0, 25.0, 26.0],
        [20.0, 45.0, 28.0],
        [-15.0, 30.0, 25.0],
        [-25.0, -5.0, 24.0],
        [-5.0, -30.0, 26.0],
        [30.0, -20.0, 27.0],
    ],
}


# ==============================================================================
# Simulation World & Physics Engine (Bubble 3: SITL World Simulator)
# ==============================================================================
class SITLWorldEngine:
    """Core physics and guidance simulator running at 60 FPS.
    Handles vectorized boid flocking, 4-zone safety, tether dynamics,
    10 Hz comms with LKP failsafe, and autonomic succession."""
    """Core physics and guidance simulator running at 60 FPS.
    Handles vectorized boid flocking, 4-zone safety, tether dynamics,
    10 Hz comms with LKP failsafe, and autonomic succession."""

    def __init__(
        self,
        drone_count: int = 8,
        enable_packet_loss: bool = False,
        seed: Optional[int] = None,
    ):
        if seed is not None:
            np.random.seed(seed)

        self.sim_time: float = 0.0
        self.last_heartbeat_broadcast: float = -1.0
        self.enable_packet_loss: bool = enable_packet_loss
        self.comms_mode: str = "TETHERED_RELAY"  # "TETHERED_RELAY", "LEADER_FOLLOWER", "GOSSIP_CONSENSUS"
        self.rf_jamming_rate: float = 0.0        # Electronic warfare RF jamming rate [0.0, 0.5]

        # Global Mission State
        self.formation_mode: str = "ECHELON_ESCORT"  # ECHELON_ESCORT, NATURAL_FLOCK, ORBIT_VORTEX, PERIMETER_PATROL
        self.tether_anchor: np.ndarray = np.copy(TETHER_ANCHOR)
        self.tether_severed: bool = False

        # Bubble 1: Tactical C2 Gateway (GCS Intent & Mission Layer)
        self.c2: C2Gateway = C2Gateway(self)

        # Fleet Doctrine A (Lost-Link Failsafe Loiter & Automated RTB) (TKT-16)
        self.lost_link_loiter_timer: float = 0.0
        self.lost_link_policy: str = "DOCTRINE_A"
        self.swarm_rtb_active: bool = False

        # Autonomous Swarm RTB & Coordinated Soft-Landing (TKT-19)
        self.overhead_marshal_active: bool = False
        self.recovery_stage: str = "IDLE"  # "IDLE", "OVERHEAD_MARSHAL", "LANDING"
        self.landing_pads: Dict[int, int] = {}  # pad_idx -> drone_id

        # Build Drone Fleet
        self.drones: List[Drone] = []
        self._highest_drone_id: int = drone_count - 1
        self._init_fleet(drone_count)

        # Pluggable Guidance Brain Plugin (Default: LeaderFollowerBrain)
        self.active_brain_name: str = "LEADER_FOLLOWER"
        self.brain: BaseBrain = LeaderFollowerBrain(self)

    def set_brain(self, brain_name: str) -> bool:
        """Hot-swap the guidance brain plugin at runtime."""
        key = brain_name.upper()
        if key in BRAIN_REGISTRY:
            self.active_brain_name = key
            self.brain = BRAIN_REGISTRY[key](self)
            self.brain.reset()
            if key == "GOSSIP_CONSENSUS":
                self.comms_mode = "GOSSIP_CONSENSUS"
            elif key == "LEADER_FOLLOWER" and self.comms_mode == "GOSSIP_CONSENSUS":
                self.comms_mode = "LEADER_FOLLOWER"
            return True
        return False

    def set_comms_mode(self, mode_name: str) -> bool:
        """Switch comms mode: TETHERED_RELAY, LEADER_FOLLOWER, or GOSSIP_CONSENSUS."""
        mode = mode_name.upper().replace("-", "_").replace(" ", "_")
        if mode in ("TETHERED_RELAY", "TETHER_RELAY", "TETHERED"):
            self.comms_mode = "TETHERED_RELAY"
            if self.active_brain_name != "LEADER_FOLLOWER":
                self.set_brain("LEADER_FOLLOWER")
            return True
        elif mode in ("LEADER_FOLLOWER", "WIRELESS_STAR", "STAR"):
            self.comms_mode = "LEADER_FOLLOWER"
            if self.active_brain_name != "LEADER_FOLLOWER":
                self.set_brain("LEADER_FOLLOWER")
            return True
        elif mode in ("GOSSIP_CONSENSUS", "P2P_MESH", "MESH", "GOSSIP"):
            self.comms_mode = "GOSSIP_CONSENSUS"
            if self.active_brain_name != "GOSSIP_CONSENSUS":
                self.set_brain("GOSSIP_CONSENSUS")
            return True
        return False

    def set_rf_jamming(self, rate: float) -> float:
        """Set electronic warfare RF jamming rate [0.0, 0.5]."""
        self.rf_jamming_rate = float(np.clip(rate, 0.0, 0.5))
        if self.rf_jamming_rate > 0.0:
            self.enable_packet_loss = True
        return self.rf_jamming_rate

    def calculate_packet_drop_prob(self, distance: float, is_hardline: bool = False) -> float:
        """Calculate packet loss probability using distance-dependent path loss and jamming.
        P_loss(d) = gamma + (1 - gamma) * 0.25 * (d / R_max)^2"""
        if is_hardline:
            return 0.0
        if distance > RADIO_RANGE:
            return 1.0
        if not self.enable_packet_loss and self.rf_jamming_rate <= 0.0:
            return 0.0
        gamma = self.rf_jamming_rate
        norm_dist = distance / RADIO_RANGE
        prob = gamma + (1.0 - gamma) * 0.25 * (norm_dist ** 2)
        return float(np.clip(prob, 0.0, 1.0))

    def is_packet_dropped(self, distance: float, is_hardline: bool = False) -> bool:
        """Stochastically determine if packet is lost given distance and jamming rate."""
        p = self.calculate_packet_drop_prob(distance, is_hardline)
        if p <= 0.0:
            return False
        if p >= 1.0:
            return True
        return bool(np.random.random() < p)

    def calculate_rssi(self, distance: float, is_hardline: bool = False) -> Tuple[float, float]:
        """Calculate RSSI (dBm) and normalized link quality [0.0, 1.0].
        Hardline fiber: -35.0 dBm, 1.0 quality.
        Wireless: Free-space loss from -40.0 dBm at 1m with -40*gamma jamming penalty."""
        if is_hardline:
            return -35.0, 1.0
        d = max(1.0, float(distance))
        loss = 20.0 * math.log10(d)
        jamming_penalty = 40.0 * self.rf_jamming_rate
        rssi = -40.0 - loss - jamming_penalty
        rssi = float(np.clip(rssi, -115.0, -35.0))
        lq = max(0.0, min(1.0, (rssi - (-100.0)) / 60.0)) * (1.0 - self.rf_jamming_rate)
        return round(rssi, 1), round(float(lq), 2)

    def _init_fleet(self, count: int) -> None:
        """Initialize leader and followers in safe starting formation."""
        self.drones.clear()
        self.tether_severed = False
        self.lost_link_loiter_timer = 0.0
        self.swarm_rtb_active = False
        self.overhead_marshal_active = False
        self.recovery_stage = "IDLE"
        self.landing_pads.clear()
        self._highest_drone_id = count - 1

        # Leader #0 positioned at (0, 0, 20)
        leader = Drone(
            drone_id=0,
            tier=TIER_LEADER,
            pos=np.array([0.0, 0.0, 20.0], dtype=float),
            is_leader=True,
        )
        self.drones.append(leader)

        # Followers initialized in staggered grid behind leader
        for i in range(1, count):
            row = (i + 1) // 2
            side = 1.0 if (i % 2 == 1) else -1.0
            x = -row * 6.0
            y = side * row * 6.0
            z = 20.0 + (row * 0.5)
            drone = Drone(
                drone_id=i,
                tier=TIER_ESCORT,
                pos=np.array([x, y, z], dtype=float),
                is_leader=False,
            )
            self.drones.append(drone)

        for d in self.drones:
            d.acknowledged_target = np.copy(self.target_pos)

    # ==========================================================================
    # C2 Mission Gateway Delegated Properties (GCS Bubble 1)
    # ==========================================================================
    @property
    def target_pos(self) -> np.ndarray:
        """Commanded target waypoint at GCS C2 layer."""
        return self.c2.target_pos

    @target_pos.setter
    def target_pos(self, value: Union[List[float], np.ndarray]) -> None:
        self.c2.target_pos = value

    @property
    def patrol_active(self) -> bool:
        return self.c2.patrol_active

    @patrol_active.setter
    def patrol_active(self, val: bool) -> None:
        self.c2.patrol_active = val

    @property
    def patrol_speed(self) -> float:
        return self.c2.patrol_speed

    @patrol_speed.setter
    def patrol_speed(self, val: float) -> None:
        self.c2.patrol_speed = val

    @property
    def patrol_time(self) -> float:
        return self.c2.patrol_time

    @patrol_time.setter
    def patrol_time(self, val: float) -> None:
        self.c2.patrol_time = val

    @property
    def patrol_center(self) -> np.ndarray:
        return self.c2.patrol_center

    @patrol_center.setter
    def patrol_center(self, val: np.ndarray) -> None:
        self.c2.patrol_center = val

    @property
    def patrol_scale(self) -> np.ndarray:
        return self.c2.patrol_scale

    @patrol_scale.setter
    def patrol_scale(self, val: np.ndarray) -> None:
        self.c2.patrol_scale = val

    @property
    def last_gossip_inject_time(self) -> float:
        return self.c2.last_gossip_inject_time

    @last_gossip_inject_time.setter
    def last_gossip_inject_time(self, val: float) -> None:
        self.c2.last_gossip_inject_time = val

    @property
    def route_active(self) -> bool:
        return self.c2.route_active

    @route_active.setter
    def route_active(self, val: bool) -> None:
        self.c2.route_active = val

    @property
    def route_name(self) -> str:
        return self.c2.route_name

    @route_name.setter
    def route_name(self, val: str) -> None:
        self.c2.route_name = val

    @property
    def route_waypoints(self) -> List[np.ndarray]:
        return self.c2.route_waypoints

    @route_waypoints.setter
    def route_waypoints(self, val: List[np.ndarray]) -> None:
        self.c2.route_waypoints = val

    @property
    def route_loop(self) -> bool:
        return self.c2.route_loop

    @route_loop.setter
    def route_loop(self, val: bool) -> None:
        self.c2.route_loop = val

    @property
    def route_acceptance_radius(self) -> float:
        return self.c2.route_acceptance_radius

    @route_acceptance_radius.setter
    def route_acceptance_radius(self, val: float) -> None:
        self.c2.route_acceptance_radius = val

    @property
    def rng_recon_active(self) -> bool:
        return self.c2.rng_recon_active

    @rng_recon_active.setter
    def rng_recon_active(self, val: bool) -> None:
        self.c2.rng_recon_active = val

    @property
    def rng_dwelling(self) -> bool:
        return self.c2.rng_dwelling

    @rng_dwelling.setter
    def rng_dwelling(self, val: bool) -> None:
        self.c2.rng_dwelling = val

    @property
    def rng_dwell_time(self) -> float:
        return self.c2.rng_dwell_time

    @rng_dwell_time.setter
    def rng_dwell_time(self, val: float) -> None:
        self.c2.rng_dwell_time = val

    @property
    def rng_dwell_timer(self) -> float:
        return self.c2.rng_dwell_timer

    @rng_dwell_timer.setter
    def rng_dwell_timer(self, val: float) -> None:
        self.c2.rng_dwell_timer = val

    @property
    def rng_station_count(self) -> int:
        return self.c2.rng_station_count

    @rng_station_count.setter
    def rng_station_count(self, val: int) -> None:
        self.c2.rng_station_count = val

    @property
    def rng_geofence_min(self) -> np.ndarray:
        return self.c2.rng_geofence_min

    @rng_geofence_min.setter
    def rng_geofence_min(self, val: np.ndarray) -> None:
        self.c2.rng_geofence_min = val

    @property
    def rng_geofence_max(self) -> np.ndarray:
        return self.c2.rng_geofence_max

    @rng_geofence_max.setter
    def rng_geofence_max(self, val: np.ndarray) -> None:
        self.c2.rng_geofence_max = val

    @property
    def rng_min_variance(self) -> float:
        return self.c2.rng_min_variance

    @rng_min_variance.setter
    def rng_min_variance(self, val: float) -> None:
        self.c2.rng_min_variance = val

    @property
    def rng_tether_clearance(self) -> float:
        return self.c2.rng_tether_clearance

    @rng_tether_clearance.setter
    def rng_tether_clearance(self, val: float) -> None:
        self.c2.rng_tether_clearance = val

    @property
    def rng_status(self) -> str:
        return self.c2.rng_status

    @property
    def rng_paused(self) -> bool:
        return self.c2.rng_paused

    def is_link_active(self) -> bool:
        return self.c2.is_link_active()

    def get_acknowledged_target(self) -> np.ndarray:
        """Return the current onboard target acknowledged by the active swarm guidance."""
        if self.comms_mode in ("TETHERED_RELAY", "LEADER_FOLLOWER"):
            current_leader = self.leader
            if current_leader is not None:
                return np.copy(current_leader.acknowledged_target)
            return np.copy(self.target_pos)
        elif self.comms_mode == "GOSSIP_CONSENSUS":
            if hasattr(self.brain, "peer_states") and self.brain.peer_states:
                for d in self.drones:
                    if not d.crashed and d.id in self.brain.peer_states:
                        return np.copy(self.brain.peer_states[d.id].active_target)
            return np.copy(self.target_pos)
        return np.copy(self.target_pos)

    @property
    def leader(self) -> Optional[Drone]:
        """Get the active fleet leader."""
        for drone in self.drones:
            if drone.is_leader and not drone.crashed:
                return drone
        return None

    def get_2ic(self) -> Optional[Drone]:
        """Dynamically determine the Second-in-Command (2IC) drone (TKT-16).
        Highest succession score among surviving, non-leader escorts:
        score = (Tier * 1000) - ID."""
        candidates = [
            d for d in self.drones
            if not d.crashed and not d.is_leader and not d.tether_lost
        ]
        if not candidates:
            candidates = [d for d in self.drones if not d.crashed and not d.is_leader]
        if not candidates:
            return None
        candidates.sort(key=lambda d: d.succession_score(), reverse=True)
        return candidates[0]

    # ==========================================================================
    # Step Integration Loop (60 FPS) & GCS C2 Dispatcher (TKT-25)
    # ==========================================================================
    def step_c2(self, dt: float) -> None:
        """Advance GCS C2 layer: auto-patrol trajectory generation, RNG recon touch-and-go, and periodic uplink transmission."""
        self.c2.step(dt)

    def step(self, dt: float = DT, advance_c2: bool = True) -> None:
        """Advance the physical simulation world by dt seconds."""
        if advance_c2:
            self.step_c2(dt)

        self.sim_time += dt

        # 0b. Advance Doctrine A Lost-Link Loiter & Automated RTB Timer (TKT-16 / TKT-19)
        if self.tether_severed and not self.swarm_rtb_active:
            if self.lost_link_loiter_timer > 0.0:
                self.lost_link_loiter_timer = max(0.0, self.lost_link_loiter_timer - dt)
                if self.lost_link_loiter_timer <= 0.0:
                    # Loiter grace period expired: automated return to base (Stage 1 Overhead Marshal)
                    self.command_rtb_hold()

        # 0c. Advance Drone Breakaway Jettison State Machine (TKT-16 / TKT-27)
        for drone in self.drones:
            if not drone.crashed and drone.status == "BREAKAWAY_JETTISON" and not drone.tether_jettisoned:
                drone.jettison_timer += dt
                drone.jettison_watchdog = max(0.0, drone.jettison_watchdog - dt)

                # Physical Clearance Envelope Gates:
                # 1. Altitude gate: Delta Z >= 4.0m below formation or reached safety floor (max(3.0m, lead_z - 4.5m))
                current_leader = self.leader
                lead_z = current_leader.pos[2] if current_leader is not None else 20.0
                z_floor = max(3.0, lead_z - 4.5)
                alt_cleared = (drone.pos[2] <= lead_z - 4.0) or (drone.pos[2] <= z_floor + 0.25)

                # 2. Lateral gate: Delta R_perp >= 3.0m displacement from leader/formation corridor
                if current_leader is not None:
                    peel_dir = getattr(drone, "peel_off_dir", np.zeros(3, dtype=float))
                    if float(np.linalg.norm(peel_dir[:2])) > 0.01:
                        lat_offset = abs(float(np.dot(drone.pos[:2] - current_leader.pos[:2], peel_dir[:2])))
                    else:
                        lat_offset = float(np.linalg.norm(drone.pos[:2] - current_leader.pos[:2]))
                    lat_cleared = lat_offset >= 3.0
                else:
                    lat_cleared = True

                # 3. Watchdog fallback (6.0s timeout)
                watchdog_tripped = drone.jettison_watchdog <= 0.0

                if (alt_cleared and lat_cleared) or watchdog_tripped:
                    drone.tether_jettisoned = True
                    drone.peel_off_timer = 0.0
                    drone.status = "ACTIVE"

        # 0d. Advance Drone Landing Stagger Clearance Timers (TKT-19)
        for drone in self.drones:
            if not drone.crashed and drone.flight_state == "DESCENDING":
                if drone.landing_wait_timer > 0.0:
                    drone.landing_wait_timer = max(0.0, drone.landing_wait_timer - dt)
                    if drone.landing_wait_timer <= 0.0:
                        drone.landing_stage = "APPROACH"

        # 1. Comms & Packet Protocol from active Brain Plugin
        self.brain.update_comms(dt)

        # 2. Check Tether Dynamics & Severance (Tethered Relay Mode)
        if self.comms_mode == "TETHERED_RELAY":
            self._update_tether_dynamics()

        # 3. Compute Guidance Targets from active Brain Plugin
        guidance_accs = self.brain.compute_guidance(dt)

        # 4. Compute 4-Zone Separation & Relative-Velocity Evasion
        # Bounded flight controller steering effort (not god-mode physics)
        avoidance_accs = self._compute_4zone_avoidance()

        # 5. Compute Tether Clearance Repulsion (if tethered)
        tether_accs = (
            self._compute_tether_avoidance()
            if self.comms_mode == "TETHERED_RELAY"
            else {d.id: np.zeros(3, dtype=float) for d in self.drones}
        )

        # 5b. Flight State Machine Overrides (TAKEOFF, EN_ROUTE, DESCENDING, LANDED)
        for drone in self.drones:
            if drone.crashed:
                continue

            if drone.flight_state == "TAKEOFF":
                # Phase 1: Pure vertical climb to Z >= 20.0m
                # Horizontal guidance locked to 0 with active velocity braking
                # Vertical acceleration climbs smoothly up to 5.0 m/s
                target_vz = 5.0
                climb_az = float(np.clip(4.0 * (target_vz - drone.vel[2]), 0.0, 8.0))
                brake_ax = float(-4.0 * drone.vel[0])
                brake_ay = float(-4.0 * drone.vel[1])
                guidance_accs[drone.id] = np.array([brake_ax, brake_ay, climb_az], dtype=float)

                # Near launch pad, suppress horizontal avoidance so drone ascends cleanly
                if drone.pos[2] < 5.0:
                    avoidance_accs[drone.id][0] = 0.0
                    avoidance_accs[drone.id][1] = 0.0

            elif drone.flight_state == "EN_ROUTE":
                # Phase 2: Autonomous transit toward swarm formation slot or active target
                # Check arrival proximity to transition to Phase 3: CRUISE
                if self.active_brain_name == "LEADER_FOLLOWER":
                    current_leader = self.leader
                    if current_leader is not None:
                        if hasattr(self.brain, "_get_formation_slot"):
                            slot_pos = self.brain._get_formation_slot(drone, current_leader)
                            if float(np.linalg.norm(drone.pos - slot_pos)) <= 5.0:
                                drone.flight_state = "CRUISE"
                    else:
                        drone.flight_state = "CRUISE"
                elif self.active_brain_name == "GOSSIP_CONSENSUS":
                    if hasattr(self.brain, "peer_states") and drone.id in self.brain.peer_states:
                        peer_state = self.brain.peer_states[drone.id]
                        if float(np.linalg.norm(drone.pos - peer_state.active_target)) <= 15.0:
                            drone.flight_state = "CRUISE"
                else:
                    drone.flight_state = "CRUISE"

            elif drone.flight_state == "LANDED":
                guidance_accs[drone.id] = np.zeros(3, dtype=float)
                avoidance_accs[drone.id] = np.zeros(3, dtype=float)
                tether_accs[drone.id] = np.zeros(3, dtype=float)
                drone.vel = np.zeros(3, dtype=float)
                drone.acc = np.zeros(3, dtype=float)
                drone.rotor_rpm = 0.0

            elif drone.flight_state == "DESCENDING":
                if drone.landing_wait_timer > 0.0:
                    drone.landing_stage = "WAITING_CLEARANCE"
                    # Hold in overhead marshal pattern outside tether cylinder
                    target_hold_z = 22.0 + (drone.landing_pad_idx or 0) * 0.5
                    az_hold = float(np.clip(3.0 * (target_hold_z - drone.pos[2]) - 2.0 * drone.vel[2], -4.0, 4.0))
                    guidance_accs[drone.id][2] = az_hold
                else:
                    pad_pos = (
                        drone.landing_pad
                        if drone.landing_pad is not None
                        else self.get_pad_position(drone.landing_pad_idx or 0)
                    )
                    to_pad_xy = pad_pos[:2] - drone.pos[:2]
                    dist_xy = float(np.linalg.norm(to_pad_xy))

                    # 1. Horizontal Guidance toward Assigned Recovery Pad
                    if dist_xy > 0.15:
                        dir_xy = to_pad_xy / dist_xy
                        v_target_xy = min(4.0, max(0.8, dist_xy * 1.5)) * dir_xy
                        a_target_x = float(np.clip(3.5 * (v_target_xy[0] - drone.vel[0]), -6.0, 6.0))
                        a_target_y = float(np.clip(3.5 * (v_target_xy[1] - drone.vel[1]), -6.0, 6.0))
                    else:
                        a_target_x = float(-5.0 * drone.vel[0])
                        a_target_y = float(-5.0 * drone.vel[1])

                    # 2. Vertical Guidance: 2-Stage Deceleration Sink Rate
                    # Stage 1 Sink: vz = -1.5 m/s down to Z = 1.0m
                    # Stage 2 Cushion: vz = -0.5 m/s from Z = 1.0m down to Z = 0.5m
                    current_z = drone.pos[2]
                    if dist_xy > 2.5:
                        # Maintain safe approach altitude while closing horizontal distance
                        target_vz = float(np.clip(2.0 * (15.0 - current_z), -1.5, 2.0))
                        drone.landing_stage = "APPROACH"
                    elif current_z > 1.0:
                        target_vz = -1.5
                        drone.landing_stage = "APPROACH"
                    else:
                        target_vz = -0.5
                        drone.landing_stage = "FINAL_DESCENT"

                    a_target_z = float(np.clip(4.0 * (target_vz - drone.vel[2]), -6.0, 6.0))
                    guidance_accs[drone.id] = np.array([a_target_x, a_target_y, a_target_z], dtype=float)

                    # Suppress horizontal avoidance near ground (Z < 3.0m) to allow clean pad docking
                    if current_z < 3.0:
                        avoidance_accs[drone.id][0] *= 0.1
                        avoidance_accs[drone.id][1] *= 0.1

                    # Touchdown Check: Z <= 0.52m and dist_xy <= 0.35m
                    if current_z <= 0.52 and dist_xy <= 0.35:
                        drone.pos = np.copy(pad_pos)
                        drone.vel = np.zeros(3, dtype=float)
                        drone.acc = np.zeros(3, dtype=float)
                        drone.flight_state = "LANDED"
                        drone.status = "LANDED"
                        drone.landing_stage = "LANDED"
                        drone.rotor_rpm = 0.0

                        # Succession if landed drone was fleet leader
                        if drone.is_leader:
                            drone.is_leader = False
                            living_airborne = [
                                d for d in self.drones
                                if not d.crashed and d.flight_state != "LANDED" and d.id != drone.id
                            ]
                            if living_airborne:
                                successor = max(living_airborne, key=lambda d: d.succession_score())
                                successor.is_leader = True
                                successor.tier = TIER_LEADER
                                successor.status = "FLEET_LEADER"

        # 6. Integrate Kinematics (Euler) & Boundary Clamps
        for drone in self.drones:
            if drone.crashed:
                # Motor disengaged: ballistic drop
                drone.acc = np.array([0.0, 0.0, GRAVITY], dtype=float)
                drone.vel[0] *= 0.98
                drone.vel[1] *= 0.98
                drone.vel[2] += GRAVITY * dt
                drone.pos += drone.vel * dt
                drone.rotor_rpm = max(0.0, drone.rotor_rpm - 300.0)

                # Ground contact clamp
                if drone.pos[2] <= 0.0:
                    drone.pos[2] = 0.0
                    drone.vel = np.zeros(3, dtype=float)
                    drone.acc = np.zeros(3, dtype=float)
                    drone.rotor_rpm = 0.0
                continue

            if drone.flight_state == "LANDED":
                # Grounded on pad: motors disarmed, zero kinematics
                drone.acc = np.zeros(3, dtype=float)
                drone.vel = np.zeros(3, dtype=float)
                if drone.landing_pad is not None:
                    drone.pos = np.copy(drone.landing_pad)
                drone.rotor_rpm = 0.0
                continue

            # Sum and clamp accelerations
            total_acc = (
                guidance_accs[drone.id]
                + avoidance_accs[drone.id]
                + tether_accs[drone.id]
            )
            acc_mag = float(np.linalg.norm(total_acc))
            if acc_mag > A_MAX:
                total_acc = (total_acc / acc_mag) * A_MAX
            drone.acc = total_acc

            # Update velocity with clamp
            drone.vel += drone.acc * dt
            vel_mag = float(np.linalg.norm(drone.vel))
            if vel_mag > V_MAX:
                drone.vel = (drone.vel / vel_mag) * V_MAX

            # If TAKEOFF and near ground, damp horizontal velocity
            if drone.flight_state == "TAKEOFF" and drone.pos[2] < 5.0:
                drone.vel[0] *= 0.85
                drone.vel[1] *= 0.85

            # Update position
            drone.pos += drone.vel * dt

            # Transition from TAKEOFF to EN_ROUTE once Z >= 20.0m
            if drone.flight_state == "TAKEOFF" and drone.pos[2] >= 20.0:
                drone.flight_state = "EN_ROUTE"
                # If no living leader exists in the fleet, promote this drone to leader
                has_living_leader = any(
                    d.is_leader and not d.crashed for d in self.drones if d.id != drone.id
                )
                if not has_living_leader:
                    drone.is_leader = True
                    drone.tier = TIER_LEADER
                    drone.status = "FLEET_LEADER"
                    drone.flight_state = "CRUISE"

            # Strict safety floor clamp during breakaway dive (TKT-27)
            if drone.status == "BREAKAWAY_JETTISON" and not drone.tether_jettisoned:
                lead_z = self.leader.pos[2] if self.leader is not None else 20.0
                z_floor = max(3.0, lead_z - 4.5)
                if drone.pos[2] < z_floor:
                    drone.pos[2] = z_floor
                    if drone.vel[2] < 0.0:
                        drone.vel[2] = 0.0

            # Floor safety clamp (keep airborne above 0.5m)
            if drone.pos[2] < 0.5:
                drone.pos[2] = 0.5
                if drone.vel[2] < 0.0:
                    drone.vel[2] = 0.0

            # Orientation update
            speed_xy = math.hypot(drone.vel[0], drone.vel[1])
            if speed_xy > 0.2:
                target_yaw = math.atan2(drone.vel[1], drone.vel[0])
                # Smooth yaw slew
                yaw_diff = (target_yaw - drone.yaw + math.pi) % (2.0 * math.pi) - math.pi
                drone.yaw += yaw_diff * min(1.0, 8.0 * dt)

            # Bank tilt from acceleration
            drone.pitch = float(np.clip(-drone.acc[0] * 0.04, -0.4, 0.4))
            drone.roll = float(np.clip(-drone.acc[1] * 0.04, -0.4, 0.4))

            # Battery drain
            drone.battery = max(0.0, drone.battery - 0.005 * dt)

    # ==========================================================================
    # 1. Radio Comms (10 Hz Baud) & LKP Model
    # ==========================================================================
    def _update_radio_comms(self) -> None:
        """Simulate realistic 10 Hz radio broadcast packets and timeout-based
        Lost-Link state transitions."""
        current_leader = self.leader

        # Check if 10 Hz broadcast timer fired
        broadcast_tick = False
        if self.sim_time - self.last_heartbeat_broadcast >= HEARTBEAT_INTERVAL:
            self.last_heartbeat_broadcast = self.sim_time
            broadcast_tick = True

        for drone in self.drones:
            if drone.crashed:
                continue

            if drone.is_leader:
                if self.comms_mode == "TETHERED_RELAY":
                    drone.rssi, drone.link_quality = self.calculate_rssi(0.0, is_hardline=True)
                    drone.lost_link = False
                    drone.last_heartbeat_time = self.sim_time
                else:
                    # Wireless Star: Leader communicates with GCS base
                    d_gcs = float(np.linalg.norm(drone.pos - self.tether_anchor))
                    drone.rssi, drone.link_quality = self.calculate_rssi(d_gcs, is_hardline=False)
                    if broadcast_tick:
                        dropped = self.is_packet_dropped(d_gcs, is_hardline=False)
                        in_range = d_gcs <= RADIO_RANGE
                        if in_range and not dropped:
                            drone.last_heartbeat_time = self.sim_time
                            drone.lost_link = False
                            drone.lkp = np.copy(drone.pos)
                            if drone.status == "LOST_LINK":
                                drone.status = "FLEET_LEADER"
                    if self.sim_time - drone.last_heartbeat_time > LINK_TIMEOUT:
                        drone.lost_link = True
                        drone.status = "LOST_LINK"
                continue

            # Follower receiving leader heartbeat
            if current_leader is not None:
                dist_to_leader = float(np.linalg.norm(drone.pos - current_leader.pos))
                drone.rssi, drone.link_quality = self.calculate_rssi(dist_to_leader, is_hardline=False)

                if broadcast_tick:
                    in_range = dist_to_leader <= RADIO_RANGE
                    packet_dropped = self.is_packet_dropped(dist_to_leader, is_hardline=False)

                    if in_range and not packet_dropped:
                        drone.lkp = np.copy(current_leader.pos)
                        drone.last_heartbeat_time = self.sim_time
                        drone.lost_link = False
                        if drone.status == "LOST_LINK":
                            drone.status = "ACTIVE"
            else:
                drone.rssi, drone.link_quality = -115.0, 0.0

            # Check lost-link timeout (1.0 second threshold)
            if self.sim_time - drone.last_heartbeat_time > LINK_TIMEOUT:
                drone.lost_link = True
                drone.status = "LOST_LINK"

    # ==========================================================================
    # 2. Tether Avoidance & Severance Dynamics
    # ==========================================================================
    def _update_tether_dynamics(self) -> None:
        """Check cable length limits and physical collisions by follower airframes.
        Physical contact <= 1.0m snaps cable and triggers autonomic succession."""
        if self.comms_mode != "TETHERED_RELAY":
            return
        current_leader = self.leader
        if current_leader is None or self.tether_severed:
            return

        # Check maximum cable length extension
        cable_vec = current_leader.pos - self.tether_anchor
        cable_len = float(np.linalg.norm(cable_vec))
        if cable_len > TETHER_MAX_LEN:
            self._snap_tether(current_leader, reason="TENSILE_SNAP_MAX_LENGTH")
            return

        # Check collision with cable by any follower
        for drone in self.drones:
            if drone.crashed or drone.is_leader:
                continue

            d_perp, _ = self._dist_to_segment(
                drone.pos, self.tether_anchor, current_leader.pos
            )
            if d_perp <= TETHER_SNAP_DIST:
                self._snap_tether(
                    current_leader,
                    reason=f"FOLLOWER_COLLISION_DRONE_{drone.id}",
                )
                break

    def _snap_tether(self, leader_drone: Drone, reason: str = "") -> None:
        """Snap tether, initiate umbilical emergency breakaway dive on leader,
        dynamically promote 2IC, and initiate Doctrine A loiter timer (TKT-16 / TKT-27)."""
        self.tether_severed = True
        leader_drone.tether_lost = True
        leader_drone.jettison_watchdog = 6.0
        leader_drone.jettison_timer = 0.0
        leader_drone.tether_jettisoned = False
        leader_drone.status = "BREAKAWAY_JETTISON"

        # Abort mission route in Tethered Relay mode (TKT-28)
        self.route_active = False
        for d in self.drones:
            d.mission_active = False

        # Demote leader and promote dynamic 2IC
        self._trigger_autonomic_succession(demote_old_leader=True)

        # Engage Fleet Doctrine A: 30s defensive loiter before automated RTB
        self.lost_link_loiter_timer = 30.0
        self.swarm_rtb_active = False

    def _compute_tether_avoidance(self) -> Dict[int, np.ndarray]:
        """Apply lateral repulsion when followers enter the 3.0m cable clearance envelope."""
        accs = {d.id: np.zeros(3, dtype=float) for d in self.drones}
        if self.comms_mode != "TETHERED_RELAY":
            return accs
        current_leader = self.leader

        if current_leader is None or self.tether_severed:
            return accs

        for drone in self.drones:
            if drone.crashed or drone.is_leader:
                continue

            d_perp, r_perp = self._dist_to_segment(
                drone.pos, self.tether_anchor, current_leader.pos
            )
            if d_perp < TETHER_AVOID_DIST:
                if d_perp < 0.01:
                    # Coincident with cable: push in positive X
                    push_dir = np.array([1.0, 0.0, 0.0], dtype=float)
                else:
                    push_dir = r_perp / d_perp

                mag = 15.0 * ((TETHER_AVOID_DIST - d_perp) / TETHER_AVOID_DIST)
                accs[drone.id] = push_dir * mag

        return accs

    @staticmethod
    def _dist_to_segment(
        point: np.ndarray, seg_a: np.ndarray, seg_b: np.ndarray
    ) -> Tuple[float, np.ndarray]:
        """Calculate Euclidean distance and perpendicular offset vector from a point
        to a line segment [seg_a, seg_b]."""
        ab = seg_b - seg_a
        ab_len_sq = float(np.dot(ab, ab))
        if ab_len_sq < 1e-6:
            r = point - seg_a
            return float(np.linalg.norm(r)), r

        t = float(np.clip(np.dot(point - seg_a, ab) / ab_len_sq, 0.0, 1.0))
        closest = seg_a + (t * ab)
        r_perp = point - closest
        return float(np.linalg.norm(r_perp)), r_perp

    # ==========================================================================
    # 3. Guidance Targets & Formation Brain
    # ==========================================================================
    def _compute_guidance_accelerations(self) -> Dict[int, np.ndarray]:
        """Compute guidance steering forces toward target waypoint, formation slots,
        or Last Known Position (LKP) if link is lost."""
        accs = {d.id: np.zeros(3, dtype=float) for d in self.drones}
        current_leader = self.leader

        for drone in self.drones:
            if drone.crashed:
                continue

            # 1. Leader Guidance: Track global target waypoint
            if drone.is_leader:
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, self.target_pos, max_speed=V_CRUISE
                )
                continue

            # 2. Lost-Link Guidance: Autonomous navigation toward LKP
            if drone.lost_link:
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, drone.lkp, max_speed=V_LKP
                )
                continue

            # 3. Active Follower Formation Guidance
            if current_leader is not None:
                slot_pos = self._get_formation_slot(drone, current_leader)
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, slot_pos, max_speed=V_CRUISE
                )
            else:
                # No surviving leader: loiter in place
                accs[drone.id] = -drone.vel * 2.0

        return accs

    def _get_formation_slot(self, drone: Drone, leader: Drone) -> np.ndarray:
        """Calculate the desired 3D slot coordinate for a follower based on the active formation mode."""
        if hasattr(self, "brain") and hasattr(self.brain, "_get_formation_slot"):
            return self.brain._get_formation_slot(drone, leader)

        # Fallback to local calculation if brain not initialized
        living_followers = [
            d for d in self.drones if not d.crashed and not d.is_leader
        ]
        try:
            k = living_followers.index(drone) + 1
        except ValueError:
            k = drone.id

        if self.formation_mode == "ECHELON_ESCORT":
            # Staggered V-wedge behind leader
            speed_xy = math.hypot(leader.vel[0], leader.vel[1])
            if speed_xy > 0.5:
                heading = np.array(
                    [leader.vel[0] / speed_xy, leader.vel[1] / speed_xy, 0.0]
                )
            else:
                heading = np.array(
                    [math.cos(leader.yaw), math.sin(leader.yaw), 0.0]
                )
            lateral = np.array([-heading[1], heading[0], 0.0])

            row = (k + 1) // 2
            side = 1.0 if (k % 2 == 1) else -1.0
            return (
                leader.pos
                - (row * 6.0 * heading)
                + (side * row * 6.0 * lateral)
                + np.array([0.0, 0.0, row * 0.5])
            )

        elif self.formation_mode == "ORBIT_VORTEX":
            # Helical cylinder revolving around leader
            radius = 14.0
            omega = 0.8
            angle = omega * self.sim_time + (2.0 * math.pi * (k - 1) / max(1, len(living_followers)))
            z_offset = 3.0 * math.sin(2.0 * angle)
            return (
                leader.pos
                + np.array([radius * math.cos(angle), radius * math.sin(angle), z_offset])
            )

        elif self.formation_mode == "PERIMETER_PATROL":
            # Static defensive sentry ring holding equidistant guard stations around target
            radius = 25.0
            angle = (2.0 * math.pi * (k - 1)) / max(1, len(living_followers))
            return (
                self.target_pos
                + np.array([radius * math.cos(angle), radius * math.sin(angle), 0.0])
            )

        else:
            # NATURAL_FLOCK default: follow behind leader with minor offset
            row = (k + 1) // 2
            side = 1.0 if (k % 2 == 1) else -1.0
            return leader.pos + np.array([-row * 5.0, side * 5.0, 0.0])

    @staticmethod
    def _steer_toward(
        current_pos: np.ndarray,
        current_vel: np.ndarray,
        target_pos: np.ndarray,
        max_speed: float,
        arrival_radius: float = 6.0,
    ) -> np.ndarray:
        """Reynolds Arrival Steering behavior: decelerates smoothly as target is approached."""
        offset = target_pos - current_pos
        dist = float(np.linalg.norm(offset))
        if dist < 0.25:  # 0.25m arrival deadband stops micro-hunting/jitter
            return -current_vel * 3.0

        if dist < arrival_radius:
            desired_speed = max_speed * (dist / arrival_radius)
        else:
            desired_speed = max_speed

        desired_vel = (offset / dist) * desired_speed
        steering_force = (desired_vel - current_vel) * 2.5
        return steering_force

    # ==========================================================================
    # 4. Sane 4-Zone Spatial Safety (Relative-Velocity Damping)
    # ==========================================================================
    def _compute_4zone_avoidance(self) -> Dict[int, np.ndarray]:
        """Compute pairwise collision avoidance across the 4 physical zones:
        - Zone 1 (d > 8.0m): Nominal clear, 0 force.
        - Zone 2 (3.5m < d <= 8.0m): Linear separation push.
        - Zone 3 (1.5m < d <= 3.5m): Critical push + active relative-velocity braking.
        - Zone 4 (d <= 1.5m): Prop clash, motors killed, ballistic crash."""
        accs = {d.id: np.zeros(3, dtype=float) for d in self.drones}
        n = len(self.drones)

        for i in range(n):
            drone_i = self.drones[i]
            if drone_i.crashed or drone_i.flight_state == "LANDED":
                continue

            for j in range(i + 1, n):
                drone_j = self.drones[j]
                if drone_j.crashed or drone_j.flight_state == "LANDED":
                    continue

                diff = drone_i.pos - drone_j.pos
                dist = float(np.linalg.norm(diff))

                # Check Zone 4: Physical Contact / Prop Clash (d <= 1.5m)
                if dist <= ZONE4_CLASH:
                    self._trigger_crash(drone_i, reason=f"CLASH_WITH_DRONE_{drone_j.id}")
                    self._trigger_crash(drone_j, reason=f"CLASH_WITH_DRONE_{drone_i.id}")
                    continue

                # Normal vector from j to i
                if dist < 0.01:
                    n_ij = np.array([1.0, 0.0, 0.0], dtype=float)
                else:
                    n_ij = diff / dist

                # Check Zone 3: Critical Danger (1.5m < d <= 3.5m)
                if dist <= ZONE3_OUTER:
                    # Evasive push magnitude
                    push_mag = 12.0 + 8.0 * ((ZONE3_OUTER - dist) / (ZONE3_OUTER - ZONE4_CLASH))
                    a_push_i = push_mag * n_ij
                    a_push_j = -push_mag * n_ij

                    # Relative-velocity damping (brakes incoming momentum)
                    # v_rel = v_i - v_j. Project onto n_ij.
                    v_rel = drone_i.vel - drone_j.vel
                    v_closing = float(np.dot(v_rel, n_ij))
                    if v_closing < 0.0:
                        # Approaching each other: apply opposing braking force
                        a_damp_i = -1.5 * (v_closing * n_ij)
                        a_damp_j = 1.5 * (v_closing * n_ij)
                    else:
                        a_damp_i = np.zeros(3, dtype=float)
                        a_damp_j = np.zeros(3, dtype=float)

                    accs[drone_i.id] += a_push_i + a_damp_i
                    accs[drone_j.id] += a_push_j + a_damp_j

                # Check Zone 2: Caution Buffer (3.5m < d <= 8.0m)
                elif dist <= ZONE2_OUTER:
                    sep_mag = 5.0 * ((ZONE2_OUTER - dist) / (ZONE2_OUTER - ZONE3_OUTER))
                    accs[drone_i.id] += sep_mag * n_ij
                    accs[drone_j.id] -= sep_mag * n_ij

        return accs

    def _trigger_crash(self, drone: Drone, reason: str = "") -> None:
        """Disengage drone motors and trigger ballistic tumble."""
        drone.crashed = True
        drone.flight_state = "CRASHED"
        drone.status = "CRASHED"
        was_leader = drone.is_leader
        if (drone.is_leader or drone.id == 0) and self.comms_mode == "TETHERED_RELAY":
            if not self.tether_severed:
                self.tether_severed = True
                drone.tether_lost = True
                drone.jettison_timer = 0.0
                drone.jettison_watchdog = 0.0
                drone.tether_jettisoned = True
                if self.lost_link_loiter_timer <= 0.0:
                    self.lost_link_loiter_timer = 30.0
                    self.swarm_rtb_active = False
        if was_leader:
            drone.is_leader = False
            self._trigger_autonomic_succession(demote_old_leader=True, retired_leader=drone)
        drone.mission_active = False

    # ==========================================================================
    # 5. Autonomic Leadership Succession
    # ==========================================================================
    def _trigger_autonomic_succession(
        self, demote_old_leader: bool = True, retired_leader: Optional[Drone] = None
    ) -> Optional[Drone]:
        """Elect the highest-scoring surviving follower to take over fleet guidance.
        Succession Score = (HardwareTier * 1000) - DroneID."""
        candidates = [
            d for d in self.drones
            if not d.crashed and not d.is_leader and not d.tether_lost
        ]
        if not candidates:
            # Fallback to any living drone if all tethered
            candidates = [d for d in self.drones if not d.crashed and not d.is_leader]

        if not candidates:
            return None

        # Sort by succession score descending
        candidates.sort(key=lambda d: d.succession_score(), reverse=True)
        new_leader = candidates[0]
        old_leader: Optional[Drone] = retired_leader

        # Demote previous leader
        if demote_old_leader:
            for drone in self.drones:
                if drone.id != new_leader.id and drone.is_leader:
                    old_leader = drone
                    drone.is_leader = False
                    if not drone.crashed:
                        if getattr(drone, "tether_lost", False) and not getattr(drone, "tether_jettisoned", False):
                            drone.status = "BREAKAWAY_JETTISON"
                        else:
                            drone.status = "ACTIVE"
                        # Lateral Peel-Off Corridor Deconfliction (TKT-15)
                        speed_xy = math.hypot(drone.vel[0], drone.vel[1])
                        if speed_xy > 0.5:
                            heading = np.array([drone.vel[0] / speed_xy, drone.vel[1] / speed_xy, 0.0], dtype=float)
                        else:
                            heading = np.array([math.cos(drone.yaw), math.sin(drone.yaw), 0.0], dtype=float)
                        lat = np.array([-heading[1], heading[0], 0.0], dtype=float)

                        # Determine relative lateral displacement of promoted successor
                        rel_pos = new_leader.pos - drone.pos
                        d_lat = float(np.dot(rel_pos, lat))

                        # Peel away from successor's approach side (anti-jousting)
                        peel_dir = -lat if d_lat > 0.0 else lat
                        norm_peel = float(np.linalg.norm(peel_dir))
                        if norm_peel > 0.01:
                            peel_dir = peel_dir / norm_peel
                        else:
                            peel_dir = np.array([0.0, 1.0, 0.0], dtype=float)

                        drone.peel_off_timer = 2.5
                        drone.peel_off_dir = peel_dir

        # Promote new leader
        new_leader.is_leader = True
        new_leader.tier = TIER_LEADER
        new_leader.status = "PROMOTED_LEADER"
        new_leader.lost_link = False
        new_leader.promotion_timer = 1.5
        if old_leader is not None:
            new_leader.acknowledged_target = np.copy(old_leader.acknowledged_target)
            new_leader.mission_waypoints = [np.copy(wp) for wp in old_leader.mission_waypoints]
            new_leader.current_wp_idx = old_leader.current_wp_idx
            new_leader.mission_active = old_leader.mission_active
            new_leader.mission_loop = old_leader.mission_loop
            new_leader.wp_loiter_timer = getattr(old_leader, "wp_loiter_timer", 0.0)
            new_leader.wp_loitering = getattr(old_leader, "wp_loitering", False)

        # Notify active brain hook of succession
        if hasattr(self, "brain") and hasattr(self.brain, "on_succession"):
            self.brain.on_succession(old_leader, new_leader)

        return new_leader

    # ==========================================================================
    # 6. Mission Controls & Casualty Laboratory
    # ==========================================================================
    @staticmethod
    def get_pad_position(pad_idx: int) -> np.ndarray:
        """Calculate coordinates for recovery/launch pad index.
        Base pad layout outside the 3.0m tether clearance envelope.
        Centered at (0.0, -6.0, 0.5) with multi-drone grid layout (>= 2.5m separation)."""
        pad_x = float((pad_idx % 4 - 1.5) * 2.5)
        pad_y = float(-6.0 - (pad_idx // 4) * 3.0)
        return np.array([pad_x, pad_y, 0.5], dtype=float)

    def command_rtb_hold(self) -> None:
        """Stage 1 Recovery: Commanded RTB (Hold).
        Fleet transits to Base (0, 0, 25m) at cruise speed and settles into
        a stationary sentry ring at Z=20-25m outside 3m tether clearance cylinder."""
        self.overhead_marshal_active = True
        self.swarm_rtb_active = True
        self.recovery_stage = "OVERHEAD_MARSHAL"
        self.patrol_active = False
        self.set_target([0.0, 0.0, 25.0])
        for drone in self.drones:
            if not drone.crashed and drone.flight_state not in ("LANDED", "DESCENDING"):
                drone.flight_state = "OVERHEAD_HOLD"
                drone.status = "OVERHEAD_HOLD"

    def cancel_rtb(self) -> None:
        """Cancel RTB / overhead marshaling and return living airborne drones to mission cruise."""
        self.overhead_marshal_active = False
        self.swarm_rtb_active = False
        self.lost_link_loiter_timer = 0.0
        self.recovery_stage = "IDLE"
        if self.tether_severed and self.comms_mode == "TETHERED_RELAY":
            self.comms_mode = "LEADER_FOLLOWER"
        for drone in self.drones:
            if not drone.crashed and drone.flight_state in ("OVERHEAD_HOLD", "DESCENDING"):
                drone.flight_state = "CRUISE"
                drone.status = "FLEET_LEADER" if drone.is_leader else "ACTIVE"

    def command_land_swarm(self) -> List[int]:
        """Stage 2 Recovery: Operator-authorized coordinated landing.
        Engages ATC pad reservation and sequential staggered descent
        (2.0s clearance intervals to eliminate prop wash down-drafts)."""
        airborne_drones = [
            d for d in self.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING")
        ]
        if not airborne_drones:
            return []

        airborne_drones.sort(key=lambda d: d.id)
        self.recovery_stage = "LANDING"
        self.overhead_marshal_active = False

        queued_ids = []
        for i, drone in enumerate(airborne_drones):
            # Find lowest unreserved pad
            used_pads = set(self.landing_pads.keys())
            pad_idx = 0
            while pad_idx in used_pads:
                pad_idx += 1
            self.landing_pads[pad_idx] = drone.id
            drone.landing_pad_idx = pad_idx
            drone.landing_pad = self.get_pad_position(pad_idx)
            drone.landing_wait_timer = i * 2.0  # 2.0s clearance intervals
            drone.flight_state = "DESCENDING"
            drone.status = "LANDING"
            drone.landing_stage = "WAITING_CLEARANCE" if i > 0 else "APPROACH"
            queued_ids.append(drone.id)

        return queued_ids

    def command_land_drone(self, drone_id: int) -> bool:
        """Command an individual drone to land on an available recovery pad."""
        for drone in self.drones:
            if drone.id == drone_id and not drone.crashed and drone.flight_state not in ("LANDED", "DESCENDING"):
                used_pads = set(self.landing_pads.keys())
                pad_idx = 0
                while pad_idx in used_pads:
                    pad_idx += 1
                self.landing_pads[pad_idx] = drone.id
                drone.landing_pad_idx = pad_idx
                drone.landing_pad = self.get_pad_position(pad_idx)
                drone.landing_wait_timer = 0.0
                drone.flight_state = "DESCENDING"
                drone.status = "LANDING"
                drone.landing_stage = "APPROACH"
                return True
        return False

    def command_takeoff_drone(self, drone_id: int) -> bool:
        """Relaunch a landed drone from its pad via vertical takeoff."""
        for drone in self.drones:
            if drone.id == drone_id and drone.flight_state == "LANDED":
                drone.flight_state = "TAKEOFF"
                drone.status = "ACTIVE"
                drone.landing_stage = "NONE"
                drone.rotor_rpm = 5200.0
                drone.vel = np.zeros(3, dtype=float)
                drone.acc = np.zeros(3, dtype=float)
                if drone.landing_pad_idx is not None and drone.landing_pad_idx in self.landing_pads:
                    del self.landing_pads[drone.landing_pad_idx]
                drone.landing_pad_idx = None
                drone.landing_pad = None
                return True
        return False

    def launch_reinforcement(self) -> Drone:
        """Deploy a new reinforcement drone from the base launch pad.
        Assigns the next sequential drone ID (max_id + 1).
        Spawns on the base launch pad, climbing vertically to safe cruise altitude (Z >= 20m)
        before transitioning to EN_ROUTE and joining the active formation."""
        current_max = max((d.id for d in self.drones), default=-1)
        self._highest_drone_id = max(getattr(self, "_highest_drone_id", -1), current_max) + 1
        new_id = self._highest_drone_id

        # Base launch pad coordinates outside the 3.0m tether clearance envelope
        slot_idx = new_id % 8
        spawn_pos = self.get_pad_position(slot_idx)

        drone = Drone(
            drone_id=new_id,
            tier=TIER_ESCORT,
            pos=spawn_pos,
            is_leader=False,
        )
        drone.flight_state = "TAKEOFF"
        drone.status = "ACTIVE"
        drone.vel = np.zeros(3, dtype=float)
        drone.acc = np.zeros(3, dtype=float)
        drone.rotor_rpm = 5200.0

        # Prime LKP and heartbeat timestamp with current leader position
        current_leader = self.leader
        if current_leader is not None:
            drone.lkp = np.copy(current_leader.pos)
        else:
            drone.lkp = np.copy(self.target_pos)
        drone.last_heartbeat_time = self.sim_time

        self.drones.append(drone)

        # Register with active Gossip brain if needed
        if hasattr(self.brain, "peer_states"):
            if hasattr(self.brain, "_get_peer_state"):
                self.brain._get_peer_state(new_id)

        return drone

    def kill_leader(self) -> None:
        """Trigger emergency motor cut on active leader."""
        current_leader = self.leader
        if current_leader is not None:
            self._trigger_crash(current_leader, reason="OPERATOR_KILL_COMMAND")

    def kill_drone(self, drone_id: int) -> None:
        """Kill motors on a specific drone by ID."""
        for drone in self.drones:
            if drone.id == drone_id and not drone.crashed:
                self._trigger_crash(drone, reason="OPERATOR_TARGET_KILL")
                break

    def reboot_drone(self, drone_id: int) -> None:
        """Safely redeploy a crashed drone into the formation via vertical takeoff."""
        for drone in self.drones:
            if drone.id == drone_id:
                ground_pos = np.array([drone.pos[0], drone.pos[1], 0.5], dtype=float)
                drone.reset_state(ground_pos, is_leader=False, flight_state="TAKEOFF")
                break

    def reboot_all_dead(self) -> List[int]:
        """Reboot and revive all crashed casualties in-place from ground via vertical takeoff
        without resetting surviving drones, simulation clock, or active targets."""
        revived_ids: List[int] = []
        crashed_drones = [d for d in self.drones if d.crashed]
        if not crashed_drones:
            return revived_ids

        has_living_leader = any(d.is_leader and not d.crashed for d in self.drones)

        for i, drone in enumerate(crashed_drones):
            make_leader = (not has_living_leader and i == 0)
            ground_pos = np.array([drone.pos[0], drone.pos[1], 0.5], dtype=float)
            # Give slight stagger if multiple crashed at the exact same location
            ground_pos[0] += (drone.id % 3 - 1) * 2.0
            ground_pos[1] += ((drone.id // 3) % 3 - 1) * 2.0

            drone.reset_state(ground_pos, is_leader=make_leader, flight_state="TAKEOFF")
            drone.status = "PROMOTED_LEADER" if make_leader else "ACTIVE"
            if make_leader:
                has_living_leader = True
            revived_ids.append(drone.id)

        return revived_ids

    def clear_crashed_wreckage(self) -> List[int]:
        """Purge all destroyed/crashed drones from the simulation.
        Returns list of removed drone IDs."""
        purged_ids = [d.id for d in self.drones if d.crashed]
        if not purged_ids:
            return []

        self.drones = [d for d in self.drones if not d.crashed]

        # Clean up peer states if in gossip consensus brain mode
        if hasattr(self.brain, "peer_states"):
            for pid in purged_ids:
                self.brain.peer_states.pop(pid, None)

        # Clean up any pads held by purged drones
        for pid in list(self.landing_pads.keys()):
            if self.landing_pads[pid] in purged_ids:
                del self.landing_pads[pid]

        # If fleet has surviving drones but no active leader, elect one
        if self.leader is None and self.drones:
            self._trigger_autonomic_succession(demote_old_leader=False)

        return purged_ids

    def reboot_fleet(self) -> None:
        """Reset and reboot all drones back to nominal starting states."""
        self._init_fleet(len(self.drones))
        self.lost_link_loiter_timer = 0.0
        self.swarm_rtb_active = False
        self.overhead_marshal_active = False
        self.recovery_stage = "IDLE"
        self.landing_pads.clear()
        self.brain.reset()

    def cut_tether(self) -> None:
        """Operator manual tether severance directive."""
        current_leader = self.leader
        if current_leader is not None:
            self._snap_tether(current_leader, reason="OPERATOR_SEVER_TETHER")

    def set_formation(self, mode: str) -> None:
        """Switch active formation flight mode."""
        valid_modes = [
            "ECHELON_ESCORT",
            "NATURAL_FLOCK",
            "ORBIT_VORTEX",
            "PERIMETER_PATROL",
        ]
        if mode in valid_modes:
            self.formation_mode = mode
            if hasattr(self.brain, "formation_mode"):
                self.brain.formation_mode = mode

    def dispatch_target_uplink(self, target_pos: Union[List[float], np.ndarray]) -> Dict[str, Any]:
        """Attempt to deliver a commanded target waypoint across the simulated C2 link (tether or RF).
        If the physical channel is broken (severed tether or out of 120m RF range),
        the uplink packet is dropped and drone onboard buffers remain untouched."""
        target_arr = np.array(target_pos, dtype=float)

        # 1. TETHERED_RELAY: Hardline umbilical to Drone 0
        if self.comms_mode == "TETHERED_RELAY":
            if self.tether_severed:
                return {
                    "status": "DROPPED",
                    "reason": "TETHER_SEVERED",
                    "error": "Tactical umbilical is severed. Uplink packet dropped.",
                }
            # Hardline delivery to tethered leader
            for d in self.drones:
                if d.id == 0 or d.is_leader:
                    d.acknowledged_target = np.copy(target_arr)
            return {
                "status": "OK",
                "channel": "HARDLINE_UMBILICAL",
                "target": [round(float(v), 2) for v in target_arr],
            }

        # 2. LEADER_FOLLOWER: Wireless Star to active fleet leader
        elif self.comms_mode == "LEADER_FOLLOWER":
            current_leader = self.leader
            if current_leader is None or current_leader.crashed:
                return {
                    "status": "DROPPED",
                    "reason": "NO_LEADER",
                    "error": "No living fleet leader to receive wireless uplink.",
                }
            d_gcs = float(np.linalg.norm(current_leader.pos - self.tether_anchor))
            if d_gcs > RADIO_RANGE:
                return {
                    "status": "OUT_OF_RANGE",
                    "reason": "RADIO_RANGE_EXCEEDED",
                    "error": f"Leader Drone #{current_leader.id} ({d_gcs:.1f}m) out of GCS radio range ({RADIO_RANGE}m)",
                }
            current_leader.acknowledged_target = np.copy(target_arr)
            return {
                "status": "OK",
                "channel": "WIRELESS_STAR",
                "target": [round(float(v), 2) for v in target_arr],
            }

        # 3. GOSSIP_CONSENSUS: Wireless P2P Mesh Ingestion
        elif self.comms_mode == "GOSSIP_CONSENSUS":
            if hasattr(self.brain, "inject_target"):
                res = self.brain.inject_target(target_arr)
                if isinstance(res, dict) and res.get("status") == "OUT_OF_RANGE":
                    return res
                return res if isinstance(res, dict) else {
                    "status": "OK",
                    "channel": "P2P_MESH",
                    "target": [round(float(v), 2) for v in target_arr],
                }
            return {"status": "OK", "channel": "P2P_MESH"}

        return {"status": "ERROR", "error": f"Unknown comms mode: {self.comms_mode}"}

    def set_target(self, pos: Union[List[float], np.ndarray]) -> Dict[str, Any]:
        """Command a new 3D mission target waypoint from GCS and attempt uplink dispatch."""
        return self.c2.set_target(pos)

    def toggle_target_patrol(self, active: Optional[bool] = None) -> bool:
        """Toggle or explicitly set continuous 3D target auto-patrol loop."""
        return self.c2.toggle_target_patrol(active)

    def start_route_preset(self, preset_name: str) -> Dict[str, Any]:
        """Deploy and start a predefined route preset (TKT-28)."""
        return self.c2.start_route_preset(preset_name)

    def deploy_mission_route(
        self,
        waypoints: Union[List[List[float]], List[np.ndarray]],
        loop: bool = True,
        name: str = "CUSTOM",
    ) -> Dict[str, Any]:
        """Deploy discrete waypoint mission route to the swarm over physical comms uplink (TKT-28)."""
        return self.c2.deploy_mission_route(waypoints, loop=loop, name=name)

    def stop_mission_route(self) -> Dict[str, Any]:
        """Stop active mission route and hold current position (TKT-28)."""
        return self.c2.stop_mission_route()

    def is_target_reached(self, threshold: float = 2.5) -> bool:
        """Deterministic arrival check: whether active swarm guidance has arrived within threshold."""
        return self.c2.is_target_reached(threshold)

    def generate_safe_rng_target(self) -> np.ndarray:
        """Generate a high-variance, guaranteed-SAFE 3D target coordinate."""
        return self.c2.generate_safe_rng_target()

    def toggle_rng_recon(self, active: Optional[bool] = None) -> bool:
        """Toggle or explicitly set tactical RNG Touch-and-Go reconnaissance mode (TKT-29)."""
        return self.c2.toggle_rng_recon(active)

    def start_rng_recon(self) -> Dict[str, Any]:
        """Activate RNG touch-and-go reconnaissance mode (TKT-29)."""
        return self.c2.start_rng_recon()

    def stop_rng_recon(self) -> Dict[str, Any]:
        """Deactivate RNG touch-and-go reconnaissance mode (TKT-29)."""
        return self.c2.stop_rng_recon()

    # ==========================================================================
    # 7. Telemetry Snapshot Serialization
    # ==========================================================================
    def get_state_snapshot(self) -> Dict[str, Any]:
        """Generate high-throughput JSON-serializable dictionary for GCS and UDP."""
        current_leader = self.leader
        tether_len = 0.0
        if current_leader is not None and not self.tether_severed:
            tether_len = float(np.linalg.norm(current_leader.pos - self.tether_anchor))

        two_ic = self.get_2ic()
        two_ic_id = two_ic.id if two_ic else None

        drones_list = []
        for d in self.drones:
            d_dict = d.to_dict()
            d_dict["is_2ic"] = (d.id == two_ic_id and not d.is_leader and not d.crashed)
            drones_list.append(d_dict)

        ack_target = self.get_acknowledged_target()

        # Determine active route current waypoint index & loiter state (TKT-28 & TKT-30)
        if self.active_brain_name == "GOSSIP_CONSENSUS":
            current_wp_idx = getattr(self.brain, "current_route_wp_idx", 0)
            current_loitering = getattr(self.brain, "wp_loitering", False)
            current_loiter_timer = getattr(self.brain, "wp_loiter_timer", 0.0)
        elif current_leader is not None:
            current_wp_idx = current_leader.current_wp_idx
            current_loitering = getattr(current_leader, "wp_loitering", False)
            current_loiter_timer = getattr(current_leader, "wp_loiter_timer", 0.0)
        else:
            current_wp_idx = 0
            current_loitering = False
            current_loiter_timer = 0.0

        route_data = {
            "active": self.route_active,
            "name": self.route_name,
            "loop": self.route_loop,
            "current_wp_idx": current_wp_idx,
            "waypoints": [[round(float(v), 2) for v in wp] for wp in self.route_waypoints],
            "loitering": current_loitering,
            "loiter_timer": round(max(0.0, float(current_loiter_timer)), 1),
        }

        return {
            "sim_time": round(float(self.sim_time), 3),
            "dt": DT,
            "brain": self.active_brain_name,
            "comms_mode": self.comms_mode,
            "rf_jamming": round(float(self.rf_jamming_rate), 2),
            "gcs_radio_range": RADIO_RANGE,
            "formation": getattr(self.brain, "formation_mode", self.formation_mode),
            "target": [round(float(v), 2) for v in self.target_pos],
            "commanded_target": [round(float(v), 2) for v in self.target_pos],
            "acknowledged_target": [round(float(v), 2) for v in ack_target],
            "target_reached": self.is_target_reached(),
            "mission_route": route_data,
            "patrol_active": self.patrol_active,
            "rng_recon": {
                "active": self.rng_recon_active,
                "dwelling": self.rng_dwelling,
                "dwell_timer": round(max(0.0, float(self.rng_dwell_timer)), 1),
                "dwell_time": float(self.rng_dwell_time),
                "station_count": int(self.rng_station_count),
                "status": self.rng_status,
                "paused": self.rng_paused,
            },
            "two_ic_id": two_ic_id,
            "lost_link_loiter_timer": round(max(0.0, float(self.lost_link_loiter_timer)), 1),
            "swarm_rtb_active": self.swarm_rtb_active,
            "overhead_marshal_active": self.overhead_marshal_active,
            "recovery_stage": self.recovery_stage,
            "landing_pads": {int(k): v for k, v in self.landing_pads.items()},
            "tether": {
                "anchor": [round(float(v), 2) for v in self.tether_anchor],
                "leader_id": current_leader.id if current_leader else -1,
                "severed": self.tether_severed,
                "length": round(tether_len, 2),
                "deployed": (self.comms_mode == "TETHERED_RELAY"),
                "visible": (self.comms_mode == "TETHERED_RELAY" and not self.tether_severed),
            },
            "drones": drones_list,
            "brain_telemetry": self.brain.get_telemetry_extra(),
        }


# ==============================================================================
# Backward-Compatible Class Alias
# ==============================================================================
SwarmSimulator = SITLWorldEngine

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
    "SITLWorldEngine",
    "SwarmSimulator",
]


if __name__ == "__main__":
    import argparse
    import time
    from SIM.backend.mavlink_bridge import MavlinkBridge

    parser = argparse.ArgumentParser(description="SwarmTech3 SITL World Simulation Engine")
    parser.add_argument("--count", type=int, default=8, help="Number of drones")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="UDP host")
    parser.add_argument("--tx-port", type=int, default=9000, help="UDP Telemetry TX port")
    parser.add_argument("--rx-port", type=int, default=9001, help="UDP Command RX port")
    parser.add_argument("--mavlink-port", type=int, default=14550, help="UDP MAVLink port")
    args = parser.parse_args()

    engine = SITLWorldEngine(drone_count=args.count)
    bridge = MavlinkBridge(
        sim=engine,
        host=args.host,
        tx_port=args.tx_port,
        rx_port=args.rx_port,
        mavlink_port=args.mavlink_port,
    )

    print(f"[SITL] Engine active: {args.count} drones @ 60 FPS", flush=True)
    print(f"[SITL] Telemetry TX (UDP {args.tx_port}) | Command RX (UDP {args.rx_port}) | MAVLink (UDP {args.mavlink_port})", flush=True)

    try:
        while True:
            t0 = time.perf_counter()
            engine.step(DT)
            bridge.step()
            dt_step = time.perf_counter() - t0
            sleep_time = max(0.0, DT - dt_step)
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\n[SITL] Simulation engine terminated by operator.", flush=True)
    finally:
        bridge.close()


