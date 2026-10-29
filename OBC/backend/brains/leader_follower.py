"""
SwarmTech3_UpRising: Leader-Follower Guidance Brain (leader_follower.py)
Implements tethered flight behind a fleet leader, 10 Hz star-topology comms,
timeout-based Lost-Link state machine with LKP navigation, and autonomic succession.
"""

from __future__ import annotations
import math
from typing import Dict, List, Optional, Any, TYPE_CHECKING
import numpy as np

from .base import BaseBrain

if TYPE_CHECKING:
    from SIM.backend.sitl_engine import SwarmSimulator
    from OBC.backend.drone_firmware import Drone

RADIO_RANGE: float = 120.0
HEARTBEAT_INTERVAL: float = 0.1
LINK_TIMEOUT: float = 1.0
V_CRUISE: float = 12.0
V_LKP: float = 8.0
TIER_LEADER: int = 3
TIER_ESCORT: int = 2

# Echelon Anti-Jitter & Heading Slew Parameters
FREEZE_SPEED: float = 1.5        # m/s: leader speeds below this freeze echelon heading
FREEZE_DIST: float = 8.0         # meters: leader distance to target below this freezes echelon heading
DEADBAND_RADIUS: float = 0.25    # meters: arrival deadband stops micro-hunting/jitter
MAX_SLEW_RATE: float = math.radians(45.0)  # 45 deg/s = ~0.7854 rad/s slew limit


class LeaderFollowerBrain(BaseBrain):
    """Classic tactical swarm brain: Followers form up behind an active Leader.
    Comms are transmitted at 10 Hz over RF. Drones losing contact navigate
    autonomously to their Last Known Position (LKP)."""

    brain_name: str = "LEADER_FOLLOWER"

    def __init__(self, sim: SwarmSimulator):
        super().__init__(sim)
        self.last_heartbeat_broadcast: float = -1.0
        self.formation_mode: str = "ECHELON_ESCORT"  # ECHELON_ESCORT, NATURAL_FLOCK, ORBIT_VORTEX, PERIMETER_PATROL
        self.formation_heading: float = 0.0
        self.heading_initialized: bool = False
        self.assigned_slots: Dict[int, int] = {}

    def reset(self) -> None:
        self.last_heartbeat_broadcast = -1.0
        self.formation_heading = 0.0
        self.heading_initialized = False
        self.assigned_slots.clear()

    def on_succession(self, old_leader: Optional[Drone], new_leader: Drone) -> None:
        """Deconflict slot assignments during leadership transitions (TKT-15).
        Transfers the promoted leader's vacated slot to the demoted leader
        so existing followers maintain wing affinity without crossing."""
        if new_leader.id in self.assigned_slots:
            vacated_slot = self.assigned_slots.pop(new_leader.id)
            if old_leader is not None and not old_leader.crashed:
                self.assigned_slots[old_leader.id] = vacated_slot

    # ==========================================================================
    # 1. Radio Comms (10 Hz Star Broadcast) & LKP Failsafe
    # ==========================================================================
    def update_comms(self, dt: float) -> None:
        current_leader = self.sim.leader

        broadcast_tick = False
        if self.sim.sim_time - self.last_heartbeat_broadcast >= HEARTBEAT_INTERVAL:
            self.last_heartbeat_broadcast = self.sim.sim_time
            broadcast_tick = True

        for drone in self.sim.drones:
            if drone.crashed:
                continue

            if drone.is_leader:
                if self.sim.comms_mode == "TETHERED_RELAY":
                    drone.rssi, drone.link_quality = self.sim.calculate_rssi(0.0, is_hardline=True)
                    drone.lost_link = False
                    drone.last_heartbeat_time = self.sim.sim_time
                else:
                    # Wireless Star: Leader communicates with GCS base anchor
                    d_gcs = float(np.linalg.norm(drone.pos - self.sim.tether_anchor))
                    drone.rssi, drone.link_quality = self.sim.calculate_rssi(d_gcs, is_hardline=False)
                    if broadcast_tick:
                        dropped = self.sim.is_packet_dropped(d_gcs, is_hardline=False)
                        in_range = d_gcs <= RADIO_RANGE
                        if in_range and not dropped:
                            drone.last_heartbeat_time = self.sim.sim_time
                            drone.lost_link = False
                            drone.lkp = np.copy(drone.pos)
                            if drone.status == "LOST_LINK":
                                drone.status = "FLEET_LEADER"
                    if self.sim.sim_time - drone.last_heartbeat_time > LINK_TIMEOUT:
                        drone.lost_link = True
                        drone.status = "LOST_LINK"
                continue

            if current_leader is not None:
                dist = float(np.linalg.norm(drone.pos - current_leader.pos))
                drone.rssi, drone.link_quality = self.sim.calculate_rssi(dist, is_hardline=False)

                if broadcast_tick:
                    in_range = dist <= RADIO_RANGE
                    dropped = self.sim.is_packet_dropped(dist, is_hardline=False)

                    if in_range and not dropped:
                        drone.lkp = np.copy(current_leader.pos)
                        drone.last_heartbeat_time = self.sim.sim_time
                        drone.lost_link = False
                        if drone.status == "LOST_LINK":
                            drone.status = "ACTIVE"
            else:
                drone.rssi, drone.link_quality = -115.0, 0.0

            if self.sim.sim_time - drone.last_heartbeat_time > LINK_TIMEOUT:
                drone.lost_link = True
                drone.status = "LOST_LINK"

    # ==========================================================================
    # 2. Guidance Target Computation
    # ==========================================================================
    def compute_guidance(self, dt: float) -> Dict[int, np.ndarray]:
        accs = {d.id: np.zeros(3, dtype=float) for d in self.sim.drones}
        current_leader = self.sim.leader

        # 0. Smooth and freeze echelon heading based on leader kinematics
        if current_leader is not None and not current_leader.crashed:
            self._update_formation_heading(current_leader, dt)

        for drone in self.sim.drones:
            if drone.crashed or drone.flight_state in ("LANDED", "DESCENDING"):
                continue

            # 1. Leader guidance: steer toward global mission target
            if drone.is_leader:
                if drone.lost_link:
                    accs[drone.id] = self._steer_toward(
                        drone.pos, drone.vel, drone.lkp, max_speed=V_LKP
                    )
                    continue

                # Smooth acceleration ramp if newly promoted to give retiring leader corridor clearance
                if getattr(drone, "promotion_timer", 0.0) > 0.0:
                    drone.promotion_timer = max(0.0, drone.promotion_timer - dt)
                    ramp = 1.0 - (drone.promotion_timer / 1.5)
                    leader_speed = 7.0 + (V_CRUISE - 7.0) * ramp
                else:
                    leader_speed = V_CRUISE

                # Discrete Waypoint Mission Route Progression (TKT-28 & TKT-30)
                if getattr(drone, "mission_active", False) and getattr(drone, "mission_waypoints", None):
                    wps = drone.mission_waypoints
                    idx = drone.current_wp_idx
                    if 0 <= idx < len(wps):
                        raw_wp = wps[idx]
                        target_wp = np.array(raw_wp[:3], dtype=float)
                        loiter_duration = float(raw_wp[3]) if len(raw_wp) > 3 else 0.0
                        drone.acknowledged_target = target_wp
                        dist = float(np.linalg.norm(drone.pos - target_wp))

                        if dist <= self.sim.route_acceptance_radius:
                            if not getattr(drone, "wp_loitering", False) and loiter_duration > 0.0:
                                # Enter WAYPOINT_LOITER state and initialize countdown
                                drone.wp_loitering = True
                                drone.wp_loiter_timer = loiter_duration
                            elif getattr(drone, "wp_loitering", False):
                                # Count down loiter timer
                                drone.wp_loiter_timer = max(0.0, drone.wp_loiter_timer - dt)

                                # Formation Stabilization Gate: verify living escorts are settled near slots (<= 4.0m)
                                living_escorts = [
                                    f for f in self.sim.drones
                                    if not f.crashed and not f.is_leader and f.flight_state == "CRUISE" and not f.lost_link
                                ]
                                escorts_settled = True
                                for f in living_escorts:
                                    slot = self._get_formation_slot(f, drone)
                                    if float(np.linalg.norm(f.pos - slot)) > 4.0:
                                        escorts_settled = False
                                        break

                                # Advance only if timer elapsed AND formation is settled (or no active escorts)
                                if drone.wp_loiter_timer <= 0.0 and escorts_settled:
                                    drone.wp_loitering = False
                                    drone.wp_loiter_timer = 0.0
                                    if idx + 1 < len(wps):
                                        drone.current_wp_idx += 1
                                        drone.acknowledged_target = np.array(wps[drone.current_wp_idx][:3], dtype=float)
                                    elif getattr(drone, "mission_loop", True):
                                        drone.current_wp_idx = 0
                                        drone.acknowledged_target = np.array(wps[0][:3], dtype=float)
                                    else:
                                        drone.mission_active = False
                            else:
                                # Zero loiter: advance immediately
                                drone.wp_loitering = False
                                drone.wp_loiter_timer = 0.0
                                if idx + 1 < len(wps):
                                    drone.current_wp_idx += 1
                                    drone.acknowledged_target = np.array(wps[drone.current_wp_idx][:3], dtype=float)
                                elif getattr(drone, "mission_loop", True):
                                    drone.current_wp_idx = 0
                                    drone.acknowledged_target = np.array(wps[0][:3], dtype=float)
                                else:
                                    drone.mission_active = False

                        # If loitering, hold station at target_wp with gentle braking speed
                        if getattr(drone, "wp_loitering", False):
                            leader_speed = 3.0

                target_dest = getattr(drone, "acknowledged_target", self.sim.target_pos)
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, target_dest, max_speed=leader_speed
                )
                continue

            # 2. Lost-link guidance: autonomous navigation toward LKP
            if drone.lost_link:
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, drone.lkp, max_speed=V_LKP
                )
                continue

            # 2b. Breakaway Dive & Cable Jettison Guidance (TKT-16 / TKT-27)
            if drone.status == "BREAKAWAY_JETTISON" and not getattr(drone, "tether_jettisoned", False):
                # Vertical dive corridor: step down 4.0m below formation plane with strict safety floor
                lead_z = current_leader.pos[2] if current_leader is not None else drone.pos[2]
                z_floor = max(3.0, lead_z - 4.5)
                target_z = max(z_floor, lead_z - 4.0)

                # Smoothly cushion sink rate near safety floor to prevent inertia overshoot
                dist_to_floor = max(0.0, drone.pos[2] - z_floor)
                max_sink = min(6.5, max(0.8, dist_to_floor * 2.5))
                desired_vz = float(np.clip(3.5 * (target_z - drone.pos[2]), -max_sink, 2.0))
                a_dive_z = float(np.clip(5.0 * (desired_vz - drone.vel[2]), -18.0, 15.0))

                # Lateral push away from formation
                heading = np.array([math.cos(self.formation_heading), math.sin(self.formation_heading), 0.0], dtype=float)
                peel_dir = getattr(drone, "peel_off_dir", np.zeros(3, dtype=float))
                if float(np.linalg.norm(peel_dir)) < 0.01:
                    peel_dir = np.array([-heading[1], heading[0], 0.0], dtype=float)
                v_peel_target = peel_dir * 6.0
                a_peel_lat = (v_peel_target - drone.vel) * 2.5

                # Longitudinal forward speed braking
                v_long = float(np.dot(drone.vel, heading))
                a_brake_long = -v_long * 2.5 * heading

                accs[drone.id] = np.array([
                    a_peel_lat[0] + a_brake_long[0],
                    a_peel_lat[1] + a_brake_long[1],
                    a_dive_z
                ], dtype=float)
                continue

            # 2c. Tether Failsafe Policy: Autonomous RTB override if configured
            if getattr(drone, "tether_failsafe_policy", "REJOIN") == "RTB" and getattr(drone, "tether_lost", False):
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, np.array([0.0, 0.0, 25.0], dtype=float), max_speed=V_CRUISE
                )
                continue

            # 3. Lateral Peel-Off Corridor Guidance (Anti-Jousting)
            if getattr(drone, "peel_off_timer", 0.0) > 0.0 and current_leader is not None:
                drone.peel_off_timer = max(0.0, drone.peel_off_timer - dt)
                heading = np.array([math.cos(self.formation_heading), math.sin(self.formation_heading), 0.0], dtype=float)
                rel_to_leader = drone.pos - current_leader.pos
                long_dist = float(np.dot(rel_to_leader, heading))

                # Once drone has passed longitudinally behind the new leader, the jousting risk is over
                if long_dist < -2.0:
                    drone.peel_off_timer = 0.0
                else:
                    # Lateral yield push away from flight corridor
                    v_peel_target = drone.peel_off_dir * 8.0
                    a_peel_lat = (v_peel_target - drone.vel) * 3.0

                    # Vertical corridor separation (step down 1.5m below leader)
                    target_z = max(1.0, current_leader.pos[2] - 1.5)
                    z_err = target_z - drone.pos[2]
                    a_peel_z = np.clip(z_err * 3.0 - drone.vel[2] * 2.0, -8.0, 8.0)

                    # Longitudinal forward speed damping so leader passes cleanly
                    v_long = float(np.dot(drone.vel, heading))
                    a_peel_long = -v_long * 2.0 * heading

                    accs[drone.id] = np.array([
                        a_peel_lat[0] + a_peel_long[0],
                        a_peel_lat[1] + a_peel_long[1],
                        a_peel_z
                    ], dtype=float)
                    continue

            # 4. Normal active follower formation guidance
            if current_leader is not None:
                slot_pos = self._get_formation_slot(drone, current_leader)
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, slot_pos, max_speed=V_CRUISE
                )
            else:
                accs[drone.id] = -drone.vel * 2.0

        return accs

    def _update_formation_heading(self, leader: Drone, dt: float) -> None:
        """Update and smooth echelon formation heading with anti-jitter freeze
        and bounded slew rate filter (<= 45 deg/s)."""
        speed_xy = math.hypot(leader.vel[0], leader.vel[1])
        leader_target = getattr(leader, "acknowledged_target", self.sim.target_pos)
        to_target = leader_target - leader.pos
        dist_to_target = float(np.linalg.norm(to_target))

        # Initial bootstrap if heading has not been established yet
        if not self.heading_initialized:
            if speed_xy >= FREEZE_SPEED:
                self.formation_heading = math.atan2(leader.vel[1], leader.vel[0])
                self.heading_initialized = True
            else:
                dist_xy = math.hypot(to_target[0], to_target[1])
                if dist_xy > 1.0:
                    self.formation_heading = math.atan2(to_target[1], to_target[0])
                else:
                    self.formation_heading = float(leader.yaw)
                self.heading_initialized = True
            return

        patrol_active = getattr(self.sim, "patrol_active", False)

        # Anti-jitter freeze: when leader decelerates/reverses near stationary target or slows down
        if not patrol_active and (speed_xy < FREEZE_SPEED or dist_to_target < FREEZE_DIST):
            # Freeze heading at last stable cruising angle
            return

        # Cruising: target heading follows instantaneous velocity vector,
        # or points towards target beacon if leader slows down in transit
        if speed_xy >= FREEZE_SPEED:
            target_heading = math.atan2(leader.vel[1], leader.vel[0])
        elif dist_to_target > 1.0:
            target_heading = math.atan2(to_target[1], to_target[0])
        else:
            return

        # Apply bounded slew filter (<= 45 deg/s in standard cruise, <= 60 deg/s in patrol pursuit)
        effective_slew_rate = math.radians(60.0) if patrol_active else MAX_SLEW_RATE
        yaw_diff = (target_heading - self.formation_heading + math.pi) % (2.0 * math.pi) - math.pi
        max_step = effective_slew_rate * dt
        slew = float(np.clip(yaw_diff, -max_step, max_step))
        self.formation_heading = (self.formation_heading + slew + math.pi) % (2.0 * math.pi) - math.pi

    def _get_formation_slot(self, drone: Drone, leader: Drone) -> np.ndarray:
        living_followers = [
            d for d in self.sim.drones
            if not d.crashed and not d.is_leader and d.flight_state not in ("LANDED", "DESCENDING")
        ]
        living_ids = {d.id for d in living_followers}

        # Clean up dead, promoted, or landed drones from assigned slots
        for did in list(self.assigned_slots.keys()):
            if did not in living_ids:
                del self.assigned_slots[did]

        # Retrieve or assign persistent deconfliction slot
        if drone.id in self.assigned_slots:
            k = self.assigned_slots[drone.id]
        else:
            used = set(self.assigned_slots.values())
            try:
                suggested_k = living_followers.index(drone) + 1
            except ValueError:
                suggested_k = 1
            if suggested_k not in used:
                k = suggested_k
            else:
                k = 1
                while k in used:
                    k += 1
            self.assigned_slots[drone.id] = k

        # Stage 1 Overhead Marshal: Equidistant holding ring around base anchor (0, 0, 25m)
        if getattr(self.sim, "overhead_marshal_active", False):
            radius = 20.0
            angle = (2.0 * math.pi * (k - 1)) / max(1, len(living_followers))
            return np.array([radius * math.cos(angle), radius * math.sin(angle), 25.0], dtype=float)

        if self.formation_mode == "ECHELON_ESCORT":
            heading = np.array(
                [math.cos(self.formation_heading), math.sin(self.formation_heading), 0.0]
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
            radius = 14.0
            omega = 0.8
            angle = omega * self.sim.sim_time + (2.0 * math.pi * (k - 1) / max(1, len(living_followers)))
            z_offset = 3.0 * math.sin(2.0 * angle)
            return (
                leader.pos
                + np.array([radius * math.cos(angle), radius * math.sin(angle), z_offset])
            )

        elif self.formation_mode == "PERIMETER_PATROL":
            # Static defensive sentry ring holding equidistant guard stations around target
            radius = 25.0
            angle = (2.0 * math.pi * (k - 1)) / max(1, len(living_followers))
            center = (
                leader.acknowledged_target
                if hasattr(leader, "acknowledged_target")
                else self.sim.target_pos
            )
            return (
                center
                + np.array([radius * math.cos(angle), radius * math.sin(angle), 0.0])
            )

        else:
            # NATURAL_FLOCK
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
        deadband: float = DEADBAND_RADIUS,
    ) -> np.ndarray:
        offset = target_pos - current_pos
        dist = float(np.linalg.norm(offset))
        if dist < deadband:
            return -current_vel * 3.0

        if dist < arrival_radius:
            desired_speed = max_speed * (dist / arrival_radius)
        else:
            desired_speed = max_speed

        desired_vel = (offset / dist) * desired_speed
        return (desired_vel - current_vel) * 2.5

    # ==========================================================================
    # 3. Brain Command Hook
    # ==========================================================================
    def handle_command(self, cmd_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        cmd = cmd_dict.get("cmd", "").upper()
        if cmd == "SET_FORMATION":
            mode = cmd_dict.get("mode", "ECHELON_ESCORT")
            valid = ["ECHELON_ESCORT", "NATURAL_FLOCK", "ORBIT_VORTEX", "PERIMETER_PATROL"]
            if mode in valid:
                self.formation_mode = mode
                return {"status": "OK", "formation": mode}
        return None

    def get_telemetry_extra(self) -> Dict[str, Any]:
        return {
            "brain_mode": "LEADER_FOLLOWER",
            "formation": self.formation_mode,
            "formation_heading_deg": round(math.degrees(self.formation_heading) % 360.0, 1),
        }
