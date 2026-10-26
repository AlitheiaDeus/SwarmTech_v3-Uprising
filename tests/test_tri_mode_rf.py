"""
SwarmTech3_UpRising: Tri-Mode Architecture & Contested RF Channel Tests (test_tri_mode_rf.py)
Automated verification for TKT-17 & PLN-03:
- Tri-mode switching (TETHERED_RELAY, LEADER_FOLLOWER, GOSSIP_CONSENSUS).
- Hardline umbilical vs. wireless RF star vs. decentralized P2P mesh.
- Dynamic tether visibility & avoidance bypass in wireless modes.
- Distance-dependent RF path loss and operator electronic warfare jamming.
- GCS 120m radio uplink horizon and gateway range verification.
- Telemetry RSSI and normalized link quality tracking.
"""

from __future__ import annotations
import unittest
import numpy as np
from typing import Dict, Any

from backend.sim_core import SwarmSimulator, RADIO_RANGE, TETHER_ANCHOR
from backend.web_server import SwarmWebServer
from backend.mavlink_bridge import MavlinkBridge


class TestTriModeRF(unittest.TestCase):
    """Test suite for Tri-Mode swarm architecture and contested RF channel model."""

    def setUp(self):
        self.sim = SwarmSimulator(drone_count=5, enable_packet_loss=False, seed=42)

    def test_mode_switching_and_brain_sync(self):
        """Verify seamless switching between TETHERED_RELAY, LEADER_FOLLOWER, and GOSSIP_CONSENSUS."""
        self.assertEqual(self.sim.comms_mode, "TETHERED_RELAY")
        self.assertEqual(self.sim.active_brain_name, "LEADER_FOLLOWER")

        # Switch to WIRELESS STAR (LEADER_FOLLOWER)
        ok = self.sim.set_comms_mode("LEADER_FOLLOWER")
        self.assertTrue(ok)
        self.assertEqual(self.sim.comms_mode, "LEADER_FOLLOWER")
        self.assertEqual(self.sim.active_brain_name, "LEADER_FOLLOWER")

        # Switch to P2P MESH (GOSSIP_CONSENSUS)
        ok = self.sim.set_comms_mode("GOSSIP_CONSENSUS")
        self.assertTrue(ok)
        self.assertEqual(self.sim.comms_mode, "GOSSIP_CONSENSUS")
        self.assertEqual(self.sim.active_brain_name, "GOSSIP_CONSENSUS")

        # Switch back to TETHERED_RELAY
        ok = self.sim.set_comms_mode("TETHERED_RELAY")
        self.assertTrue(ok)
        self.assertEqual(self.sim.comms_mode, "TETHERED_RELAY")
        self.assertEqual(self.sim.active_brain_name, "LEADER_FOLLOWER")

        # Invalid mode returns False
        self.assertFalse(self.sim.set_comms_mode("INVALID_MODE"))

    def test_tether_visibility_and_dynamics_bypass_in_wireless_modes(self):
        """In wireless modes (LEADER_FOLLOWER, GOSSIP_CONSENSUS), tether is invisible and
        cable clearance forces / severance are completely bypassed."""
        # 1. Tethered Relay Mode: Tether visible and active
        self.sim.set_comms_mode("TETHERED_RELAY")
        snap = self.sim.get_state_snapshot()
        self.assertTrue(snap["tether"]["visible"])

        # 2. Wireless Star Mode: Tether invisible
        self.sim.set_comms_mode("LEADER_FOLLOWER")
        snap = self.sim.get_state_snapshot()
        self.assertFalse(snap["tether"]["visible"])

        # Place Drone 1 directly coincident with the virtual cable line between base (0,0,1.5) and leader (0,0,20)
        follower = self.sim.drones[1]
        follower.pos = np.array([0.0, 0.0, 10.0], dtype=float)

        # Run physics step: should NOT sever tether and should NOT apply clearance repulsion
        self.sim.step(0.016)
        self.assertFalse(self.sim.tether_severed)
        avoidance = self.sim._compute_tether_avoidance()
        np.testing.assert_allclose(avoidance[follower.id], np.zeros(3))

        # 3. P2P Mesh Mode: Tether invisible
        self.sim.set_comms_mode("GOSSIP_CONSENSUS")
        snap = self.sim.get_state_snapshot()
        self.assertFalse(snap["tether"]["visible"])

    def test_physics_based_rf_path_loss_and_jamming(self):
        """Verify distance-dependent path loss and EW jamming formula:
        P_loss(d) = gamma + (1 - gamma) * 0.25 * (d / R_max)^2."""
        # 1. Hardline fiber: 0% loss regardless of distance or jamming
        self.sim.set_rf_jamming(0.5)
        prob_hardline = self.sim.calculate_packet_drop_prob(distance=100.0, is_hardline=True)
        self.assertEqual(prob_hardline, 0.0)

        rssi_hardline, lq_hardline = self.sim.calculate_rssi(distance=100.0, is_hardline=True)
        self.assertEqual(rssi_hardline, -35.0)
        self.assertEqual(lq_hardline, 1.0)

        # 2. Wireless without jamming (gamma = 0.0)
        self.sim.set_rf_jamming(0.0)
        self.sim.enable_packet_loss = True

        # At d = 0m: loss = 0.0
        self.assertAlmostEqual(self.sim.calculate_packet_drop_prob(0.0), 0.0, places=4)

        # At d = 60m (half range): loss = 0.25 * (0.5)^2 = 0.0625 (6.25%)
        self.assertAlmostEqual(self.sim.calculate_packet_drop_prob(60.0), 0.0625, places=4)

        # At d = 120m (edge of range): loss = 0.25 * (1.0)^2 = 0.25 (25%)
        self.assertAlmostEqual(self.sim.calculate_packet_drop_prob(120.0), 0.25, places=4)

        # At d > 120m: loss = 1.0 (100% out of range)
        self.assertEqual(self.sim.calculate_packet_drop_prob(121.0), 1.0)

        # 3. Contested RF with Jamming (gamma = 0.3)
        self.sim.set_rf_jamming(0.3)
        # At d = 0m: loss = 0.3 (30%)
        self.assertAlmostEqual(self.sim.calculate_packet_drop_prob(0.0), 0.3, places=4)
        # At d = 120m: loss = 0.3 + 0.7 * 0.25 = 0.475 (47.5%)
        self.assertAlmostEqual(self.sim.calculate_packet_drop_prob(120.0), 0.475, places=4)

        # Jamming clamp to 0.5
        self.sim.set_rf_jamming(0.9)
        self.assertEqual(self.sim.rf_jamming_rate, 0.5)

        # 4. RSSI model
        self.sim.set_rf_jamming(0.0)
        rssi_1m, _ = self.sim.calculate_rssi(1.0)
        self.assertEqual(rssi_1m, -40.0)
        rssi_10m, _ = self.sim.calculate_rssi(10.0)
        self.assertEqual(rssi_10m, -60.0)
        rssi_100m, _ = self.sim.calculate_rssi(100.0)
        self.assertEqual(rssi_100m, -80.0)

    def test_wireless_leader_lost_link_gcs_horizon(self):
        """In LEADER_FOLLOWER mode, leader communicates with GCS over RF.
        Exceeding 120m GCS horizon triggers lost link on leader."""
        self.sim.set_comms_mode("LEADER_FOLLOWER")
        leader = self.sim.leader
        self.assertIsNotNone(leader)

        # Run 0.2s to establish initial heartbeat
        for _ in range(12):
            self.sim.step()
        self.assertFalse(leader.lost_link)

        # Teleport leader outside GCS horizon (140m > 120m)
        leader.pos = np.array([140.0, 0.0, 25.0], dtype=float)

        # Advance 1.2s (> 1.0s LINK_TIMEOUT = 72 ticks)
        for _ in range(75):
            self.sim.step()

        self.assertTrue(leader.lost_link)
        self.assertEqual(leader.status, "LOST_LINK")

        # Teleport leader back within GCS horizon (< 120m)
        leader.pos = np.array([30.0, 0.0, 25.0], dtype=float)
        for _ in range(12):
            self.sim.step()

        self.assertFalse(leader.lost_link)

    def test_gcs_target_dispatch_horizon_checks(self):
        """GCS target dispatch verifies active drone presence within 120m radio horizon."""
        # 1. LEADER_FOLLOWER: Leader outside 120m horizon rejects dispatch
        self.sim.set_comms_mode("LEADER_FOLLOWER")
        leader = self.sim.leader
        leader.pos = np.array([150.0, 0.0, 25.0], dtype=float)

        res = self.sim.set_target([150.0, 50.0, 30.0])
        self.assertEqual(res["status"], "OUT_OF_RANGE")

        # Bring leader inside horizon (< 120m)
        leader.pos = np.array([50.0, 0.0, 25.0], dtype=float)
        res = self.sim.set_target([60.0, 40.0, 30.0])
        self.assertEqual(res["status"], "OK")

        # 2. GOSSIP_CONSENSUS: Mesh requires at least one node within 120m of Base
        self.sim.set_comms_mode("GOSSIP_CONSENSUS")

        # Move all drones out to 150m
        for d in self.sim.drones:
            d.pos = np.array([150.0, d.id * 5.0, 25.0], dtype=float)

        res = self.sim.set_target([150.0, 80.0, 30.0])
        self.assertEqual(res["status"], "OUT_OF_RANGE")

        # Bring just one drone (Drone 2) within GCS range
        self.sim.drones[2].pos = np.array([30.0, 0.0, 20.0], dtype=float)
        res = self.sim.set_target([150.0, 80.0, 30.0])
        self.assertEqual(res["status"], "OK")
        self.assertEqual(res["seed_drone_id"], 2)

    def test_telemetry_snapshot_contains_rf_and_mode_data(self):
        """Snapshot must include comms_mode, rf_jamming, gcs_radio_range, and per-drone RSSI."""
        self.sim.set_comms_mode("LEADER_FOLLOWER")
        self.sim.set_rf_jamming(0.25)

        snap = self.sim.get_state_snapshot()
        self.assertEqual(snap["comms_mode"], "LEADER_FOLLOWER")
        self.assertEqual(snap["rf_jamming"], 0.25)
        self.assertEqual(snap["gcs_radio_range"], 120.0)

        # Check drone serialized RSSI
        d0 = snap["drones"][0]
        self.assertIn("rssi", d0)
        self.assertIn("link_quality", d0)
        self.assertIsInstance(d0["rssi"], (int, float))
        self.assertIsInstance(d0["link_quality"], (int, float))

    def test_network_bridge_commands(self):
        """Verify web_server and mavlink_bridge handle SET_COMMS_MODE and SET_RF_JAMMING."""
        ws_server = SwarmWebServer(self.sim)
        mav_bridge = MavlinkBridge(self.sim, enable_mavlink=False)

        # 1. SET_COMMS_MODE via WebServer
        res = ws_server.execute_command({"cmd": "SET_COMMS_MODE", "mode": "WIRELESS_STAR"})
        self.assertEqual(res["status"], "OK")
        self.assertEqual(res["comms_mode"], "LEADER_FOLLOWER")

        # 2. SET_RF_JAMMING via WebServer
        res = ws_server.execute_command({"cmd": "SET_RF_JAMMING", "rate": 0.35})
        self.assertEqual(res["status"], "OK")
        self.assertAlmostEqual(res["rf_jamming"], 0.35, places=2)

        # 3. SET_COMMS_MODE via MavlinkBridge
        res = mav_bridge.execute_command({"cmd": "SET_COMMS_MODE", "mode": "P2P_MESH"})
        self.assertEqual(res["status"], "OK")
        self.assertEqual(res["comms_mode"], "GOSSIP_CONSENSUS")

        # 4. Out of range dispatch via MavlinkBridge
        for d in self.sim.drones:
            d.pos = np.array([200.0, d.id * 5.0, 25.0], dtype=float)
        res = mav_bridge.execute_command({"cmd": "SET_TARGET", "pos": [200.0, 50.0, 30.0]})
        self.assertEqual(res["status"], "OUT_OF_RANGE")
        mav_bridge.close()


if __name__ == "__main__":
    unittest.main()
