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

import numpy as np

from backend.brains import BaseBrain, BRAIN_REGISTRY, LeaderFollowerBrain, GossipConsensusBrain


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


# ==============================================================================
# Drone Airframe Representation
# ==============================================================================
class Drone:
    """Represents an autonomous quadcopter agent with physical kinematics,
    telemetry state, comms link, and navigation brain."""

    def __init__(
        self,
        drone_id: int,
        tier: int = TIER_ESCORT,
        pos: Optional[np.ndarray] = None,
        is_leader: bool = False,
    ):
        self.id: int = drone_id
        self.tier: int = tier
        self.is_leader: bool = is_leader

        # Kinematics
        self.pos: np.ndarray = (
            np.array(pos, dtype=float) if pos is not None else np.zeros(3, dtype=float)
        )
        self.vel: np.ndarray = np.zeros(3, dtype=float)
        self.acc: np.ndarray = np.zeros(3, dtype=float)

        # Orientation & Visuals
        self.yaw: float = 0.0          # Radians (0 = +X axis)
        self.pitch: float = 0.0        # Radians
        self.roll: float = 0.0         # Radians
        self.rotor_rpm: float = 5200.0 # Visual rotor speed

        # Comms & Fault Tolerance
        self.crashed: bool = False
        self.lost_link: bool = False
        self.lkp: np.ndarray = np.copy(self.pos)
        self.last_heartbeat_time: float = 0.0

        # Status & Battery
        self.battery: float = 100.0
        self.status: str = "ACTIVE" if not is_leader else "FLEET_LEADER"
        self.tether_lost: bool = False

    def reset_state(self, pos: np.ndarray, is_leader: bool = False) -> None:
        """Reset drone state for relaunch or reboot."""
        self.pos = np.array(pos, dtype=float)
        self.vel = np.zeros(3, dtype=float)
        self.acc = np.zeros(3, dtype=float)
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.rotor_rpm = 5200.0
        self.crashed = False
        self.lost_link = False
        self.lkp = np.copy(self.pos)
        self.last_heartbeat_time = 0.0
        self.battery = 100.0
        self.is_leader = is_leader
        self.status = "FLEET_LEADER" if is_leader else "ACTIVE"
        self.tether_lost = False

    def succession_score(self) -> int:
        """Calculate autonomic succession eligibility score: (Tier * 1000) - ID."""
        return (self.tier * 1000) - self.id

    def to_dict(self) -> Dict[str, Any]:
        """Serialize state for WebSocket JSON telemetry frame."""
        return {
            "id": self.id,
            "tier": self.tier,
            "is_leader": self.is_leader,
            "pos": [round(float(v), 2) for v in self.pos],
            "vel": [round(float(v), 2) for v in self.vel],
            "yaw": round(float(self.yaw), 3),
            "pitch": round(float(self.pitch), 3),
            "roll": round(float(self.roll), 3),
            "crashed": self.crashed,
            "lost_link": self.lost_link,
            "lkp": [round(float(v), 2) for v in self.lkp],
            "battery": round(float(self.battery), 1),
            "status": self.status,
            "rotor_rpm": round(float(self.rotor_rpm), 0),
        }


# ==============================================================================
# Simulation World & Physics Engine
# ==============================================================================
class SwarmSimulator:
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

        # Global Mission State
        self.formation_mode: str = "ECHELON_ESCORT"  # ECHELON_ESCORT, NATURAL_FLOCK, ORBIT_VORTEX, PERIMETER_PATROL
        self.target_pos: np.ndarray = np.array([60.0, 40.0, 30.0], dtype=float)
        self.tether_anchor: np.ndarray = np.copy(TETHER_ANCHOR)
        self.tether_severed: bool = False

        # Build Drone Fleet
        self.drones: List[Drone] = []
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
            return True
        return False

    def _init_fleet(self, count: int) -> None:
        """Initialize leader and followers in safe starting formation."""
        self.drones.clear()
        self.tether_severed = False

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

    @property
    def leader(self) -> Optional[Drone]:
        """Get the active fleet leader."""
        for drone in self.drones:
            if drone.is_leader and not drone.crashed:
                return drone
        return None

    # ==========================================================================
    # Step Integration Loop (60 FPS)
    # ==========================================================================
    def step(self, dt: float = DT) -> None:
        """Advance the simulation world by dt seconds."""
        self.sim_time += dt

        # 1. Comms & Packet Protocol from active Brain Plugin
        self.brain.update_comms(dt)

        # 2. Check Tether Dynamics & Severance (LeaderFollowerBrain)
        if self.active_brain_name == "LEADER_FOLLOWER":
            self._update_tether_dynamics()

        # 3. Compute Guidance Targets from active Brain Plugin
        guidance_accs = self.brain.compute_guidance(dt)

        # 4. Compute 4-Zone Separation & Relative-Velocity Evasion
        # Bounded flight controller steering effort (not god-mode physics)
        avoidance_accs = self._compute_4zone_avoidance()

        # 5. Compute Tether Clearance Repulsion (if tethered)
        tether_accs = (
            self._compute_tether_avoidance()
            if self.active_brain_name == "LEADER_FOLLOWER"
            else {d.id: np.zeros(3, dtype=float) for d in self.drones}
        )

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

            # Update position
            drone.pos += drone.vel * dt

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
                drone.lost_link = False
                drone.last_heartbeat_time = self.sim_time
                continue

            # Follower receiving leader heartbeat
            if broadcast_tick and current_leader is not None:
                dist_to_leader = float(np.linalg.norm(drone.pos - current_leader.pos))
                in_range = dist_to_leader <= RADIO_RANGE

                # Stochastic packet drop (if enabled, 1% baseline)
                packet_dropped = self.enable_packet_loss and (np.random.random() < 0.01)

                if in_range and not packet_dropped:
                    drone.lkp = np.copy(current_leader.pos)
                    drone.last_heartbeat_time = self.sim_time
                    drone.lost_link = False
                    if drone.status == "LOST_LINK":
                        drone.status = "ACTIVE"

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
        """Snap tether, cut leader primary link, and trigger autonomic succession."""
        self.tether_severed = True
        leader_drone.tether_lost = True
        self._trigger_autonomic_succession(demote_old_leader=True)

    def _compute_tether_avoidance(self) -> Dict[int, np.ndarray]:
        """Apply lateral repulsion when followers enter the 3.0m cable clearance envelope."""
        accs = {d.id: np.zeros(3, dtype=float) for d in self.drones}
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
        # Follower order index among living followers
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
            if drone_i.crashed:
                continue

            for j in range(i + 1, n):
                drone_j = self.drones[j]
                if drone_j.crashed:
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
        drone.status = "CRASHED"
        if drone.is_leader:
            drone.is_leader = False
            self._trigger_autonomic_succession(demote_old_leader=True)

    # ==========================================================================
    # 5. Autonomic Leadership Succession
    # ==========================================================================
    def _trigger_autonomic_succession(self, demote_old_leader: bool = True) -> Optional[Drone]:
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

        # Demote previous leader
        if demote_old_leader:
            for drone in self.drones:
                if drone.id != new_leader.id and drone.is_leader:
                    drone.is_leader = False
                    if not drone.crashed:
                        drone.status = "ACTIVE"

        # Promote new leader
        new_leader.is_leader = True
        new_leader.tier = TIER_LEADER
        new_leader.status = "PROMOTED_LEADER"
        new_leader.lost_link = False
        return new_leader

    # ==========================================================================
    # 6. Mission Controls & Casualty Laboratory
    # ==========================================================================
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
        """Safely redeploy a crashed drone into the formation."""
        for drone in self.drones:
            if drone.id == drone_id:
                # Find spawn offset near leader or anchor
                ref_pos = self.leader.pos if self.leader is not None else np.array([0.0, 0.0, 20.0])
                spawn_pos = ref_pos + np.array([-15.0 - (drone_id * 2.0), 0.0, 5.0])
                drone.reset_state(spawn_pos, is_leader=False)
                break

    def reboot_fleet(self) -> None:
        """Reset and reboot all drones back to nominal starting states."""
        self._init_fleet(len(self.drones))
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

    def set_target(self, pos: List[float]) -> None:
        """Set global 3D mission target waypoint."""
        self.target_pos = np.array(pos, dtype=float)
        if hasattr(self.brain, "inject_target"):
            self.brain.inject_target(pos)

    # ==========================================================================
    # 7. Telemetry Snapshot Serialization
    # ==========================================================================
    def get_state_snapshot(self) -> Dict[str, Any]:
        """Generate high-throughput JSON-serializable dictionary for GCS and UDP."""
        current_leader = self.leader
        tether_len = 0.0
        if current_leader is not None and not self.tether_severed:
            tether_len = float(np.linalg.norm(current_leader.pos - self.tether_anchor))

        return {
            "sim_time": round(float(self.sim_time), 3),
            "dt": DT,
            "brain": self.active_brain_name,
            "formation": getattr(self.brain, "formation_mode", self.formation_mode),
            "target": [round(float(v), 2) for v in self.target_pos],
            "tether": {
                "anchor": [round(float(v), 2) for v in self.tether_anchor],
                "leader_id": current_leader.id if current_leader else -1,
                "severed": self.tether_severed,
                "length": round(tether_len, 2),
            },
            "drones": [d.to_dict() for d in self.drones],
            "brain_telemetry": self.brain.get_telemetry_extra(),
        }
