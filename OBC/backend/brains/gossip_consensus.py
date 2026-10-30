"""
SwarmTech3_UpRising: P2P Gossip Consensus Brain (gossip_consensus.py)
Implements decentralized Peer-to-Peer (P2P) mesh communications,
multi-hop target gossip propagation, quorum barrier consensus,
and synchronized bank transitions without a single point of failure.
"""

from __future__ import annotations
import math
from typing import Dict, List, Optional, Set, Any, TYPE_CHECKING
import numpy as np

from .base import BaseBrain

if TYPE_CHECKING:
    from SIM.backend.sitl_engine import SwarmSimulator
    from OBC.backend.drone_firmware import Drone

RADIO_RANGE: float = 120.0
HEARTBEAT_INTERVAL: float = 0.1
V_CRUISE: float = 12.0
V_LKP: float = 6.0
QUORUM_RATIO: float = 0.75  # 75% consensus threshold required before transition


DEADBAND_RADIUS: float = 0.35
MAX_SLEW_RATE: float = math.radians(45.0)


class DronePeerState:
    """Decentralized state tracked on each individual drone."""

    def __init__(self, drone_id: int, initial_target: np.ndarray):
        self.drone_id: int = drone_id
        self.active_target: np.ndarray = np.array(initial_target, dtype=float)
        self.active_seq: int = 1
        self.pending_target: np.ndarray = np.array(initial_target, dtype=float)
        self.pending_seq: int = 1
        self.echo_ack_peers: Set[int] = {drone_id}
        self.gossip_status: str = "COORDINATED"  # COORDINATED, PROPAGATING
        self.last_tx_time: float = 0.0


class GossipConsensusBrain(BaseBrain):
    """Decentralized P2P Gossip Consensus Swarm Brain.
    - No single fleet leader required.
    - Mission targets injected into any drone in range are echoed across the mesh.
    - Drones continue safe current trajectories while echoing (no sudden collision stops).
    - Once quorum (>= 75%) is reached, all drones smoothly bank toward the new target.
    - TKT-11: Structured decentralized swarm formations (Sentry Shell, Orbit Vortex,
      Chevron Echelon, and Dispersed Lattice) computed deterministically from peer ID rankings."""

    brain_name: str = "GOSSIP_CONSENSUS"

    def __init__(self, sim: SwarmSimulator):
        super().__init__(sim)
        self.current_seq: int = 1
        self.peer_states: Dict[int, DronePeerState] = {}
        self.last_broadcast_tick: float = -1.0
        self.formation_mode: str = getattr(sim, "formation_mode", "PERIMETER_PATROL")
        self.formation_heading: float = 0.0
        self.heading_initialized: bool = False
        self.route_waypoints: List[np.ndarray] = []
        self.current_route_wp_idx: int = 0
        self.route_active: bool = False
        self.route_loop: bool = True
        self.wp_loiter_timer: float = 0.0
        self.wp_loitering: bool = False
        self.reset()

    def reset(self) -> None:
        self.current_seq = 1
        self.peer_states.clear()
        self.formation_heading = 0.0
        self.heading_initialized = False
        self.route_waypoints.clear()
        self.current_route_wp_idx = 0
        self.route_active = False
        self.route_loop = True
        self.wp_loiter_timer = 0.0
        self.wp_loitering = False
        for drone in self.sim.drones:
            self.peer_states[drone.id] = DronePeerState(
                drone.id, self.sim.target_pos
            )

    def on_route_deployed(self, clean_wps: List[np.ndarray], loop: bool = True) -> None:
        """Receive newly uploaded discrete mission route (TKT-28 & TKT-30)."""
        self.route_waypoints = [np.copy(wp) for wp in clean_wps]
        self.current_route_wp_idx = 0
        self.route_active = True
        self.route_loop = loop
        self.wp_loiter_timer = 0.0
        self.wp_loitering = False
        self.current_seq += 1
        first_wp = clean_wps[0][:3]
        for drone in self.sim.drones:
            drone.mission_waypoints = [np.copy(wp) for wp in clean_wps]
            drone.current_wp_idx = 0
            drone.mission_active = True
            drone.mission_loop = loop
            drone.acknowledged_target = np.copy(first_wp)
            drone.wp_loiter_timer = 0.0
            drone.wp_loitering = False
            if drone.id in self.peer_states:
                state = self.peer_states[drone.id]
                state.pending_seq = self.current_seq
                state.pending_target = np.copy(first_wp)
                state.active_seq = self.current_seq
                state.active_target = np.copy(first_wp)
                state.echo_ack_peers = {d.id for d in self.sim.drones if not d.crashed}
                state.gossip_status = "COORDINATED"

    # ==========================================================================
    # 1. 10 Hz P2P Mesh Broadcast & Gossip Propagation
    # ==========================================================================
    def update_comms(self, dt: float) -> None:
        current_time = self.sim.sim_time

        # Check 10 Hz broadcast timer
        if current_time - self.last_broadcast_tick < HEARTBEAT_INTERVAL:
            return
        self.last_broadcast_tick = current_time

        active_drones = [
            d for d in self.sim.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING")
        ]
        living_count = len(active_drones)
        if living_count == 0:
            return

        # Build 10 Hz heartbeat packets for all active drones
        packets = {}
        for drone in active_drones:
            state = self._get_peer_state(drone.id)
            packets[drone.id] = {
                "id": drone.id,
                "pos": np.copy(drone.pos),
                "vel": np.copy(drone.vel),
                "active_seq": state.active_seq,
                "echo_seq": state.pending_seq,
                "echo_target": np.copy(state.pending_target),
            }

        # Compute RSSI & Link Quality for each drone based on proximity to nearest peer or GCS anchor
        for drone in active_drones:
            distances = [float(np.linalg.norm(drone.pos - self.sim.tether_anchor))]
            for peer in active_drones:
                if peer.id != drone.id:
                    distances.append(float(np.linalg.norm(drone.pos - peer.pos)))
            min_dist = min(distances) if distances else 0.0
            drone.rssi, drone.link_quality = self.sim.calculate_rssi(min_dist, is_hardline=False)

        # Simulate decentralized P2P RF broadcast over Euclidean RADIO_RANGE (120m)
        for drone in active_drones:
            drone_state = self._get_peer_state(drone.id)

            for peer in active_drones:
                if peer.id == drone.id:
                    continue

                dist = float(np.linalg.norm(drone.pos - peer.pos))
                if dist > RADIO_RANGE:
                    continue  # Out of radio range

                # Physics-based packet drop from path loss & EW jamming
                if self.sim.is_packet_dropped(dist, is_hardline=False):
                    continue

                peer_pkt = packets[peer.id]

                # Check if peer has newer target directive
                if peer_pkt["echo_seq"] > drone_state.pending_seq:
                    # Adopt newer pending target and begin echoing
                    drone_state.pending_seq = peer_pkt["echo_seq"]
                    drone_state.pending_target = np.copy(peer_pkt["echo_target"])
                    drone_state.echo_ack_peers = {drone.id, peer.id}
                    drone_state.gossip_status = "PROPAGATING"

                elif peer_pkt["echo_seq"] == drone_state.pending_seq:
                    # Peer is acknowledging the same pending sequence
                    drone_state.echo_ack_peers.add(peer.id)

            # Check Quorum Barrier Synchronization
            # Once quorum ratio reached, transition active target
            living_peer_ids = {d.id for d in active_drones}
            acks_in_range = drone_state.echo_ack_peers.intersection(living_peer_ids)
            ack_ratio = len(acks_in_range) / float(living_count)

            if drone_state.pending_seq > drone_state.active_seq:
                if ack_ratio >= QUORUM_RATIO:
                    # Consensus achieved: synchronized transition!
                    drone_state.active_seq = drone_state.pending_seq
                    drone_state.active_target = np.copy(drone_state.pending_target)
                    drone_state.gossip_status = "COORDINATED"

    def _get_peer_state(self, drone_id: int) -> DronePeerState:
        if drone_id not in self.peer_states:
            self.peer_states[drone_id] = DronePeerState(
                drone_id, self.sim.target_pos
            )
        return self.peer_states[drone_id]

    # ==========================================================================
    # 2. Decentralized Guidance Vector Computation (TKT-11 Formations)
    # ==========================================================================
    def compute_guidance(self, dt: float) -> Dict[int, np.ndarray]:
        """Compute guidance steering forces:
        1. Mission Arrival Steering toward consensus geometric slot offset.
        2. Local Flocking Cohesion & Alignment with nearby peers."""
        accs = {d.id: np.zeros(3, dtype=float) for d in self.sim.drones}
        active_drones = [
            d for d in self.sim.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING")
        ]
        living_count = len(active_drones)
        if living_count == 0:
            return accs

        # Check Mission Route waypoint advancement for Gossip Quorum (TKT-28 & TKT-30)
        if self.route_active and self.route_waypoints:
            wps = self.route_waypoints
            idx = self.current_route_wp_idx
            if 0 <= idx < len(wps):
                raw_wp = wps[idx]
                target_wp = np.array(raw_wp[:3], dtype=float)
                loiter_duration = float(raw_wp[3]) if len(raw_wp) > 3 else 0.0
                centroid = np.mean([d.pos for d in active_drones], axis=0)
                dist_centroid = float(np.linalg.norm(centroid - target_wp))

                if dist_centroid <= self.sim.route_acceptance_radius:
                    if not self.wp_loitering and loiter_duration > 0.0:
                        self.wp_loitering = True
                        self.wp_loiter_timer = loiter_duration
                    elif self.wp_loitering:
                        self.wp_loiter_timer = max(0.0, self.wp_loiter_timer - dt)
                        if self.wp_loiter_timer <= 0.0:
                            self.wp_loitering = False
                            self.wp_loiter_timer = 0.0
                            if idx + 1 < len(wps):
                                self.current_route_wp_idx += 1
                                next_wp = np.array(wps[self.current_route_wp_idx][:3], dtype=float)
                            elif self.route_loop:
                                self.current_route_wp_idx = 0
                                next_wp = np.array(wps[0][:3], dtype=float)
                            else:
                                self.route_active = False
                                next_wp = target_wp

                            if self.route_active:
                                self.current_seq += 1
                                for d in active_drones:
                                    d.current_wp_idx = self.current_route_wp_idx
                                    d.acknowledged_target = np.copy(next_wp)
                                    if d.id in self.peer_states:
                                        p_state = self.peer_states[d.id]
                                        p_state.active_seq = self.current_seq
                                        p_state.pending_seq = self.current_seq
                                        p_state.active_target = np.copy(next_wp)
                                        p_state.pending_target = np.copy(next_wp)
                    else:
                        self.wp_loitering = False
                        self.wp_loiter_timer = 0.0
                        if idx + 1 < len(wps):
                            self.current_route_wp_idx += 1
                            next_wp = np.array(wps[self.current_route_wp_idx][:3], dtype=float)
                        elif self.route_loop:
                            self.current_route_wp_idx = 0
                            next_wp = np.array(wps[0][:3], dtype=float)
                        else:
                            self.route_active = False
                            next_wp = target_wp

                        if self.route_active:
                            self.current_seq += 1
                            for d in active_drones:
                                d.current_wp_idx = self.current_route_wp_idx
                                d.acknowledged_target = np.copy(next_wp)
                                if d.id in self.peer_states:
                                    p_state = self.peer_states[d.id]
                                    p_state.active_seq = self.current_seq
                                    p_state.pending_seq = self.current_seq
                                    p_state.active_target = np.copy(next_wp)
                                    p_state.pending_target = np.copy(next_wp)

        # Deterministic decentralized slot indexing based on living drone IDs
        sorted_ids = sorted([d.id for d in active_drones])

        # Update virtual swarm heading toward consensus target
        consensus_target = self.sim.get_acknowledged_target()
        self._update_formation_heading(active_drones, consensus_target, dt)

        for drone in active_drones:
            # 0. Lost-link failsafe: steer toward local LKP muster hold (TKT-41 & FIX-05)
            if getattr(drone, "lost_link", False):
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, drone.lkp, max_speed=V_LKP
                )
                continue

            state = self._get_peer_state(drone.id)
            drone.acknowledged_target = np.copy(state.active_target)
            if self.formation_mode == "NATURAL_FLOCK":
                accs[drone.id] = self._compute_boid_acceleration(
                    drone, active_drones, state.active_target
                )
            else:
                k = sorted_ids.index(drone.id)
                slot_pos = self._get_formation_slot(drone, k, living_count, state.active_target)

                # 1. Target Pursuit: Arrive steering toward decentralized geometric slot
                target_acc = self._steer_toward(
                    drone.pos, drone.vel, slot_pos, max_speed=V_CRUISE
                )

                # 2. Local Flocking Alignment (match velocity of peers within 15m)
                flock_acc = np.zeros(3, dtype=float)
                neighbor_count = 0
                for peer in active_drones:
                    if peer.id == drone.id:
                        continue
                    d = float(np.linalg.norm(drone.pos - peer.pos))
                    if d <= 15.0:
                        flock_acc += (peer.vel - drone.vel) * 0.3
                        neighbor_count += 1

                if neighbor_count > 0:
                    flock_acc /= float(neighbor_count)

                accs[drone.id] = target_acc + flock_acc

        return accs

    def _update_formation_heading(self, active_drones: List[Drone], target_pos: np.ndarray, dt: float) -> None:
        """Update virtual swarm heading vector toward consensus target with smooth slew.
        Uses mean consensus flight velocity and apex-target vector to eliminate
        pinwheel spinning when wing airframe counts are asymmetric/off-balance."""
        if not active_drones:
            return

        # Cruising velocity of the swarm
        mean_vel = np.mean([d.vel for d in active_drones], axis=0)
        speed_xy = float(math.hypot(mean_vel[0], mean_vel[1]))

        # Apex drone (slot k=0 in sorted IDs) position as geometric reference (unbiased by wings)
        sorted_drones = sorted(active_drones, key=lambda d: d.id)
        apex_drone = sorted_drones[0]
        to_target = target_pos - apex_drone.pos
        dist_to_target = float(math.hypot(to_target[0], to_target[1]))

        patrol_active = getattr(self.sim, "patrol_active", False)

        # Anti-jitter freeze: when swarm is settling, hovering near target, or decelerating
        # Freeze heading at last stable cruising angle to prevent asymmetric wing pinwheel spin!
        if not patrol_active and (speed_xy < 1.0 or dist_to_target < 2.5):
            return

        # Target heading: follow flight velocity vector during cruise, or apex-to-target when starting
        if speed_xy >= 1.0:
            desired_yaw = math.atan2(mean_vel[1], mean_vel[0])
        elif dist_to_target >= 2.5:
            desired_yaw = math.atan2(to_target[1], to_target[0])
        else:
            return

        if not self.heading_initialized:
            self.formation_heading = desired_yaw
            self.heading_initialized = True
            return

        yaw_diff = (desired_yaw - self.formation_heading + math.pi) % (2.0 * math.pi) - math.pi
        effective_slew_rate = math.radians(60.0) if patrol_active else MAX_SLEW_RATE
        max_step = effective_slew_rate * dt
        slew = float(np.clip(yaw_diff, -max_step, max_step))
        self.formation_heading = (self.formation_heading + slew + math.pi) % (2.0 * math.pi) - math.pi

    def _get_formation_slot(
        self, drone: Drone, k: int, N: int, target_pos: np.ndarray
    ) -> np.ndarray:
        """Compute 3D position of decentralized geometric slot k in [0, N-1] (TKT-11).
        Preserves wing-side continuity (odd k -> right wing, even k -> left wing)
        to prevent cross-formation reciprocal collisions."""
        if k == 0:
            azimuth = 0.0
        else:
            side = 1.0 if (k % 2 == 1) else -1.0
            pair = (k + 1) // 2
            azimuth = side * pair * (2.0 * math.pi / max(1, N))

        # Stage 1 Overhead Marshal: Equidistant holding ring around Base anchor (0, 0, 25m)
        if getattr(self.sim, "overhead_marshal_active", False):
            radius = 20.0
            return np.array([radius * math.cos(azimuth), radius * math.sin(azimuth), 25.0], dtype=float)

        if self.formation_mode == "PERIMETER_PATROL":
            # Static defensive sentry ring holding equidistant guard stations around target (R = 25m)
            radius = 25.0
            return (
                target_pos
                + np.array([radius * math.cos(azimuth), radius * math.sin(azimuth), 0.0], dtype=float)
            )

        elif self.formation_mode == "ORBIT_VORTEX":
            # Dynamic rotating ring around consensus target with altitude undulation
            radius = 20.0
            omega = 0.6
            angle = omega * self.sim.sim_time + azimuth
            z_offset = 3.0 * math.sin(2.0 * angle)
            return (
                target_pos
                + np.array([radius * math.cos(angle), radius * math.sin(angle), z_offset], dtype=float)
            )

        elif self.formation_mode == "ECHELON_ESCORT":
            # Chevron / V-formation centered on consensus target oriented along virtual heading
            heading = np.array(
                [math.cos(self.formation_heading), math.sin(self.formation_heading), 0.0], dtype=float
            )
            lateral = np.array([-heading[1], heading[0], 0.0], dtype=float)

            if k == 0:
                return target_pos

            row = (k + 1) // 2
            side = 1.0 if (k % 2 == 1) else -1.0
            return (
                target_pos
                - (row * 6.0 * heading)
                + (side * row * 6.0 * lateral)
                + np.array([0.0, 0.0, row * 0.5], dtype=float)
            )

        else:
            # NATURAL_FLOCK representative station for external inspection
            return target_pos

    def _compute_boid_acceleration(
        self,
        drone: Drone,
        active_drones: List[Drone],
        consensus_target: np.ndarray,
    ) -> np.ndarray:
        """Compute true decentralized 3D Reynolds boid flocking acceleration (TKT-23).
        Forces:
        1. Separation: Inverse-square 3D repulsion (d < 5.0m) + Zone 3 momentum damping (d < 3.5m).
        2. Alignment: P2P velocity matching with neighbors (d < 14.0m).
        3. Cohesion: Attraction toward local cluster centroid (d < 20.0m).
        4. Goal Migration: Pull toward the consensus target.
        5. Vertical Z-Space Staggering & Safety Floor (Z >= 10.0m).
        """
        # 1. Goal Migration toward consensus target
        a_goal = self._steer_toward(
            drone.pos, drone.vel, consensus_target, max_speed=V_CRUISE, arrival_radius=8.0
        )

        # 2. Separation (d < 5.0m)
        a_sep = np.zeros(3, dtype=float)
        R_SEP = 5.0
        for peer in active_drones:
            if peer.id == drone.id:
                continue
            diff = drone.pos - peer.pos
            d = float(np.linalg.norm(diff))
            if 0.001 < d < R_SEP:
                rep_strength = (R_SEP - d) / (d + 0.1)
                a_sep += (diff / d) * rep_strength * 4.0

                # 3D Z-Space staggering: if horizontally close, push vertically apart to break 2D planes
                d_xy = float(math.hypot(diff[0], diff[1]))
                if d_xy < 3.5:
                    z_sign = 1.0 if diff[2] >= 0.0 else -1.0
                    a_sep[2] += z_sign * (3.5 - d_xy) * 3.0

                # Zone 3 relative-velocity approach damping (brake incoming momentum)
                if d < 3.5:
                    v_rel = drone.vel - peer.vel
                    closing = float(np.dot(v_rel, diff / d))
                    if closing < 0:
                        a_sep += -(diff / d) * closing * 2.5

        # 3. Alignment (d < 14.0m) - velocity matching with local peers
        a_align = np.zeros(3, dtype=float)
        R_ALIGN = 14.0
        align_weight_sum = 0.0
        vel_sum = np.zeros(3, dtype=float)
        for peer in active_drones:
            if peer.id == drone.id:
                continue
            d = float(np.linalg.norm(drone.pos - peer.pos))
            if d < R_ALIGN:
                w = 1.0 - (d / R_ALIGN)
                vel_sum += peer.vel * w
                align_weight_sum += w

        if align_weight_sum > 0:
            mean_vel = vel_sum / align_weight_sum
            a_align = (mean_vel - drone.vel) * 1.5

        # 4. Cohesion (d < 20.0m) - pull toward local peer centroid
        a_coh = np.zeros(3, dtype=float)
        R_COH = 20.0
        pos_sum = np.zeros(3, dtype=float)
        coh_count = 0
        for peer in active_drones:
            if peer.id == drone.id:
                continue
            d = float(np.linalg.norm(drone.pos - peer.pos))
            if d < R_COH:
                pos_sum += peer.pos
                coh_count += 1

        if coh_count > 0:
            centroid = pos_sum / float(coh_count)
            offset_cent = centroid - drone.pos
            d_cent = float(np.linalg.norm(offset_cent))
            if d_cent > 3.0:
                a_coh = (offset_cent / d_cent) * min(V_CRUISE, d_cent * 1.2) - drone.vel
                a_coh *= 0.8

        # 5. Altitude Floor & Ceiling Guard (Anisotropic 3D)
        a_z_guard = 0.0
        if drone.pos[2] < 12.0:
            a_z_guard += max(0.0, (14.0 - drone.pos[2]) * 4.0 - drone.vel[2] * 2.0)
        elif drone.pos[2] > 40.0:
            a_z_guard -= max(0.0, (drone.pos[2] - 38.0) * 4.0 + drone.vel[2] * 2.0)

        # Weighted combination of Reynolds boid forces
        total_acc = (
            2.0 * a_goal +
            3.5 * a_sep +
            1.5 * a_align +
            1.0 * a_coh
        )
        total_acc[2] += a_z_guard

        # Clamp to physical motor acceleration limit
        norm_a = float(np.linalg.norm(total_acc))
        A_MAX = 15.0
        if norm_a > A_MAX:
            total_acc = (total_acc / norm_a) * A_MAX

        return total_acc

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
    # 3. Mission Directive Ingestion (Gossip Trigger)
    # ==========================================================================
    def inject_target(self, new_target: List[float]) -> Dict[str, Any]:
        """Ingest new target command from GCS. Delivers to nearest drone within GCS radio range (120m),
        which immediately begins echoing to its peers across the mesh."""
        new_target_arr = np.array(new_target, dtype=float)

        # Find closest active drone within GCS radio range (120m)
        active_drones = [d for d in self.sim.drones if not d.crashed]
        if not active_drones:
            return {"status": "NO_ACTIVE_DRONES", "error": "No living drones in swarm"}

        drones_in_range = []
        for d in active_drones:
            dist_to_gcs = float(np.linalg.norm(d.pos - self.sim.tether_anchor))
            if dist_to_gcs <= RADIO_RANGE:
                drones_in_range.append((dist_to_gcs, d))

        if not drones_in_range:
            return {
                "status": "OUT_OF_RANGE",
                "error": f"No active drones within GCS radio range ({RADIO_RANGE}m)",
            }

        # Seed target into closest drone to GCS anchor
        drones_in_range.sort(key=lambda item: item[0])
        seed_drone = drones_in_range[0][1]

        self.current_seq += 1
        self.sim.target_pos = new_target_arr

        state = self._get_peer_state(seed_drone.id)
        state.pending_seq = self.current_seq
        state.pending_target = np.copy(new_target_arr)
        state.echo_ack_peers = {seed_drone.id}
        state.gossip_status = "PROPAGATING"

        return {
            "status": "OK",
            "cmd": "INJECT_TARGET",
            "seq": self.current_seq,
            "seed_drone_id": seed_drone.id,
            "target": [round(float(v), 2) for v in new_target_arr],
        }

    def handle_command(self, cmd_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        cmd = cmd_dict.get("cmd", "").upper()
        if cmd in ("SET_TARGET", "DISPATCH_TARGET"):
            pos = cmd_dict.get("pos", [60.0, 40.0, 30.0])
            return self.inject_target(pos)
        elif cmd == "SET_FORMATION":
            mode = cmd_dict.get("mode", "PERIMETER_PATROL")
            if mode in ("PERIMETER_PATROL", "ECHELON_ESCORT", "ORBIT_VORTEX", "NATURAL_FLOCK"):
                self.formation_mode = mode
                return {"status": "OK", "cmd": cmd, "formation": self.formation_mode}
        return None

    def get_telemetry_extra(self) -> Dict[str, Any]:
        active_drones = [d for d in self.sim.drones if not d.crashed]
        propagating_count = 0
        for d in active_drones:
            if self._get_peer_state(d.id).gossip_status == "PROPAGATING":
                propagating_count += 1

        return {
            "brain_mode": "GOSSIP_CONSENSUS",
            "formation": self.formation_mode,
            "target_seq": self.current_seq,
            "propagating_drones": propagating_count,
            "consensus_reached": (propagating_count == 0),
        }
