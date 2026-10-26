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

from backend.brains.base import BaseBrain

if TYPE_CHECKING:
    from backend.sim_core import SwarmSimulator, Drone

RADIO_RANGE: float = 120.0
HEARTBEAT_INTERVAL: float = 0.1
V_CRUISE: float = 12.0
QUORUM_RATIO: float = 0.75  # 75% consensus threshold required before transition


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
    - Once quorum (>= 75%) is reached, all drones smoothly bank toward the new target."""

    brain_name: str = "GOSSIP_CONSENSUS"

    def __init__(self, sim: SwarmSimulator):
        super().__init__(sim)
        self.current_seq: int = 1
        self.peer_states: Dict[int, DronePeerState] = {}
        self.last_broadcast_tick: float = -1.0
        self.reset()

    def reset(self) -> None:
        self.current_seq = 1
        self.peer_states.clear()
        for drone in self.sim.drones:
            self.peer_states[drone.id] = DronePeerState(
                drone.id, self.sim.target_pos
            )

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
    # 2. Decentralized Guidance Vector Computation
    # ==========================================================================
    def compute_guidance(self, dt: float) -> Dict[int, np.ndarray]:
        """Compute guidance steering forces:
        1. Mission Arrival Steering toward consensus active target.
        2. Local Flocking Cohesion & Alignment with nearby peers."""
        accs = {d.id: np.zeros(3, dtype=float) for d in self.sim.drones}
        active_drones = [
            d for d in self.sim.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING")
        ]

        for drone in active_drones:
            state = self._get_peer_state(drone.id)

            # 1. Target Pursuit: Arrive steering toward active target
            target_acc = self._steer_toward(
                drone.pos, drone.vel, state.active_target, max_speed=V_CRUISE
            )

            # 2. Local Flocking Alignment (match velocity of peers within 20m)
            flock_acc = np.zeros(3, dtype=float)
            neighbor_count = 0
            for peer in active_drones:
                if peer.id == drone.id:
                    continue
                d = float(np.linalg.norm(drone.pos - peer.pos))
                if d <= 20.0:
                    flock_acc += (peer.vel - drone.vel) * 0.5
                    neighbor_count += 1

            if neighbor_count > 0:
                flock_acc /= float(neighbor_count)

            accs[drone.id] = target_acc + flock_acc

        return accs

    @staticmethod
    def _steer_toward(
        current_pos: np.ndarray,
        current_vel: np.ndarray,
        target_pos: np.ndarray,
        max_speed: float,
        arrival_radius: float = 8.0,
    ) -> np.ndarray:
        offset = target_pos - current_pos
        dist = float(np.linalg.norm(offset))
        if dist < 0.01:
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
        return None

    def get_telemetry_extra(self) -> Dict[str, Any]:
        active_drones = [d for d in self.sim.drones if not d.crashed]
        propagating_count = 0
        for d in active_drones:
            if self._get_peer_state(d.id).gossip_status == "PROPAGATING":
                propagating_count += 1

        return {
            "brain_mode": "GOSSIP_CONSENSUS",
            "target_seq": self.current_seq,
            "propagating_drones": propagating_count,
            "consensus_reached": (propagating_count == 0),
        }
