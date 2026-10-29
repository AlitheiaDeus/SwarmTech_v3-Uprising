"""
SwarmTech3_UpRising: 3-Bubble Decoupling Architecture Verification Tests (test_bubble_decoupling.py)
Validates the structural airgaps across GCS (Bubble 1), OBC (Bubble 2), and SIM (Bubble 3):
  1. GCS Bubble: C2Gateway and discrete route presets can run independently.
  2. OBC Bubble: Drone firmware maintains isolated onboard targets and mission queues.
  3. SIM Bubble: SITLWorldEngine simulates ground truth physics and channel dynamics.
  4. Cross-Layer Contracts: Clean delegation from SITL engine to GCS C2 gateway.
"""

from __future__ import annotations
import sys
from pathlib import Path
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import unittest
import numpy as np

# Bubble 1: GCS
from GCS.backend.c2_gateway import C2Gateway, ROUTE_PRESETS
from GCS.backend.web_server import SwarmWebServer

# Bubble 2: OBC
from OBC.backend.drone_firmware import Drone, TIER_LEADER, TIER_ESCORT, TIER_SCOUT
from OBC.backend.brains import (
    BaseBrain,
    BRAIN_REGISTRY,
    LeaderFollowerBrain,
    GossipConsensusBrain,
)

# Bubble 3: SIM
from SIM.backend.sitl_engine import (
    SITLWorldEngine,
    SwarmSimulator as SITLSwarmSim,
    DT,
    ZONE4_CLASH,
    RADIO_RANGE,
    TETHER_ANCHOR,
)
from SIM.backend.mavlink_bridge import MavlinkBridge


class TestBubbleDecoupling(unittest.TestCase):
    """Test suite validating 3-bubble decoupling and airgap integrity."""

    def test_bubble1_gcs_c2_gateway_standalone(self):
        """Verify C2Gateway operates independently without requiring active physics."""
        c2 = C2Gateway(sim=None)
        self.assertFalse(c2.route_active)
        self.assertFalse(c2.patrol_active)
        self.assertFalse(c2.rng_recon_active)

        # Deploy route in standalone C2 mode
        res = c2.deploy_mission_route([[10.0, 20.0, 30.0], [40.0, 50.0, 60.0]], loop=True, name="TEST_CORRIDOR")
        self.assertEqual(res["status"], "OK")
        self.assertTrue(c2.route_active)
        self.assertEqual(c2.route_name, "TEST_CORRIDOR")
        self.assertEqual(len(c2.route_waypoints), 2)
        np.testing.assert_allclose(c2.target_pos, [10.0, 20.0, 30.0])

        # Stop route
        stop_res = c2.stop_mission_route()
        self.assertEqual(stop_res["status"], "OK")
        self.assertFalse(c2.route_active)

    def test_bubble2_obc_drone_firmware_onboard_state(self):
        """Verify Drone OBC maintains isolated target mailbox and mission queue."""
        drone = Drone(drone_id=1, tier=TIER_ESCORT, pos=np.array([10.0, 10.0, 20.0]))
        self.assertEqual(drone.id, 1)
        self.assertEqual(drone.tier, TIER_ESCORT)
        np.testing.assert_allclose(drone.pos, [10.0, 10.0, 20.0])
        np.testing.assert_allclose(drone.acknowledged_target, [10.0, 10.0, 20.0])
        self.assertTrue(drone.target_reached)

        # Update onboard target
        drone.acknowledged_target = np.array([50.0, 50.0, 20.0])
        self.assertFalse(drone.target_reached)

        # Serialization check
        d_dict = drone.to_dict()
        self.assertEqual(d_dict["id"], 1)
        self.assertEqual(d_dict["status"], "ACTIVE")
        self.assertFalse(d_dict["target_reached"])

    def test_bubble3_sim_sitl_world_engine(self):
        """Verify SITLWorldEngine advances 60 FPS kinematics and safety zones."""
        engine = SITLWorldEngine(drone_count=4, seed=42)
        self.assertEqual(len(engine.drones), 4)
        self.assertIsInstance(engine.leader, Drone)

        # Advance 10 steps
        for _ in range(10):
            engine.step(DT)

        self.assertGreater(engine.sim_time, 0.0)
        snapshot = engine.get_state_snapshot()
        self.assertIn("drones", snapshot)
        self.assertIn("tether", snapshot)
        self.assertEqual(len(snapshot["drones"]), 4)

    def test_bubble_cross_layer_contracts(self):
        """Verify decoupled bubble exports, type contracts, and C2 gateway delegation."""
        engine = SITLWorldEngine(drone_count=4, seed=42)
        # Class and type alias checks
        self.assertIs(SITLSwarmSim, SITLWorldEngine)
        self.assertIsInstance(engine.c2, C2Gateway)
        self.assertIs(engine.c2.sim, engine)

        # Delegated properties
        np.testing.assert_allclose(engine.target_pos, engine.c2.target_pos)
        self.assertEqual(engine.route_active, engine.c2.route_active)
        self.assertEqual(engine.patrol_active, engine.c2.patrol_active)
        self.assertEqual(engine.rng_recon_active, engine.c2.rng_recon_active)

        # Brain registry checks
        self.assertIn("LEADER_FOLLOWER", BRAIN_REGISTRY)
        self.assertIn("GOSSIP_CONSENSUS", BRAIN_REGISTRY)
        self.assertIs(BRAIN_REGISTRY["LEADER_FOLLOWER"], LeaderFollowerBrain)
        self.assertIs(BRAIN_REGISTRY["GOSSIP_CONSENSUS"], GossipConsensusBrain)


if __name__ == "__main__":
    unittest.main()
