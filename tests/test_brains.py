"""
Automated Unit Tests for SwarmTech3_UpRising Pluggable Brains (test_brains.py)
Tests BaseBrain, LeaderFollowerBrain, GossipConsensusBrain, P2P echo propagation,
quorum barrier consensus, hot-swapping, and Autopilot safety control layer.
"""

import unittest
import numpy as np
import json

from backend.sim_core import SwarmSimulator, DT, A_MAX, ZONE4_CLASH
from backend.brains import BRAIN_REGISTRY, LeaderFollowerBrain, GossipConsensusBrain


class TestBrains(unittest.TestCase):
    """Test suite verifying pluggable brains and P2P consensus."""

    def setUp(self):
        self.sim = SwarmSimulator(drone_count=6, enable_packet_loss=False, seed=42)

    def test_default_brain_is_leader_follower(self):
        """Verify initial default guidance brain is LeaderFollowerBrain."""
        self.assertEqual(self.sim.active_brain_name, "LEADER_FOLLOWER")
        self.assertIsInstance(self.sim.brain, LeaderFollowerBrain)

    def test_hot_swap_to_gossip_consensus(self):
        """Verify runtime hot-swapping to GossipConsensusBrain."""
        swapped = self.sim.set_brain("GOSSIP_CONSENSUS")
        self.assertTrue(swapped)
        self.assertEqual(self.sim.active_brain_name, "GOSSIP_CONSENSUS")
        self.assertIsInstance(self.sim.brain, GossipConsensusBrain)

        # Step simulation to ensure numerical stability
        for _ in range(10):
            self.sim.step()
            for d in self.sim.drones:
                self.assertFalse(np.isnan(d.pos).any())
                self.assertFalse(np.isnan(d.vel).any())

        # Swap back to LEADER_FOLLOWER
        swapped_back = self.sim.set_brain("LEADER_FOLLOWER")
        self.assertTrue(swapped_back)
        self.assertEqual(self.sim.active_brain_name, "LEADER_FOLLOWER")
        self.assertIsInstance(self.sim.brain, LeaderFollowerBrain)

    def test_gossip_propagation_and_quorum_consensus(self):
        """Verify P2P multi-hop gossip echo wave and quorum barrier synchronization."""
        self.sim.set_brain("GOSSIP_CONSENSUS")
        brain: GossipConsensusBrain = self.sim.brain

        # Initial state: all drones coordinate around target [60, 40, 30] at seq 1
        for d in self.sim.drones:
            state = brain._get_peer_state(d.id)
            self.assertEqual(state.active_seq, 1)
            self.assertEqual(state.gossip_status, "COORDINATED")

        # Ingest new target: [150, 100, 50] (seq 2)
        res = brain.inject_target([150.0, 100.0, 50.0])
        self.assertEqual(res["status"], "OK")
        self.assertEqual(res["seq"], 2)
        seed_id = res["seed_drone_id"]

        # Seed drone is now echoing/propagating, others have not received it yet
        self.assertEqual(brain._get_peer_state(seed_id).pending_seq, 2)
        self.assertEqual(brain._get_peer_state(seed_id).gossip_status, "PROPAGATING")

        # Step 15 frames (~0.25s): covers 2 10Hz broadcast ticks so echo wave completes quorum
        for _ in range(15):
            self.sim.step()

        # All living drones should now have adopted pending_seq = 2 and reached quorum
        for d in self.sim.drones:
            state = brain._get_peer_state(d.id)
            self.assertEqual(state.active_seq, 2)
            np.testing.assert_allclose(state.active_target, [150.0, 100.0, 50.0])
            self.assertEqual(state.gossip_status, "COORDINATED")

    def test_autopilot_safety_effort_clamping(self):
        """Verify Zone 2/3 safety is computed as flight-controller control effort
        bounded by motor acceleration limit A_MAX (not god-mode physics)."""
        d1 = self.sim.drones[1]
        d2 = self.sim.drones[2]

        # Place them at 2.0m closing at high speed (isolated from drone #0 at origin)
        d1.pos = np.array([50.0, 0.0, 20.0], dtype=float)
        d2.pos = np.array([52.0, 0.0, 20.0], dtype=float)
        d1.vel = np.array([8.0, 0.0, 0.0], dtype=float)
        d2.vel = np.array([-8.0, 0.0, 0.0], dtype=float)

        self.sim.step()

        # Accelerations must be clamped to A_MAX
        self.assertLessEqual(np.linalg.norm(d1.acc), A_MAX + 1e-3)
        self.assertLessEqual(np.linalg.norm(d2.acc), A_MAX + 1e-3)

        # Neither drone has crashed because d = 2.0m > 1.5m ZONE4_CLASH
        self.assertFalse(d1.crashed)
        self.assertFalse(d2.crashed)

    def test_snapshot_includes_brain_telemetry(self):
        """Verify snapshot contains brain identifier and extra telemetry metadata."""
        snapshot = self.sim.get_state_snapshot()
        self.assertIn("brain", snapshot)
        self.assertEqual(snapshot["brain"], "LEADER_FOLLOWER")
        self.assertIn("brain_telemetry", snapshot)

        # Switch to gossip brain and verify telemetry
        self.sim.set_brain("GOSSIP_CONSENSUS")
        snapshot_gossip = self.sim.get_state_snapshot()
        self.assertEqual(snapshot_gossip["brain"], "GOSSIP_CONSENSUS")
        self.assertIn("target_seq", snapshot_gossip["brain_telemetry"])
        self.assertIn("consensus_reached", snapshot_gossip["brain_telemetry"])

    def test_echelon_heading_freeze_near_target(self):
        """Verify formation heading freezes when leader is within 8.0m of target."""
        brain: LeaderFollowerBrain = self.sim.brain
        leader = self.sim.leader

        # Position leader 20m from target, moving towards +X at 10 m/s
        self.sim.target_pos = np.array([100.0, 0.0, 20.0], dtype=float)
        leader.pos = np.array([80.0, 0.0, 20.0], dtype=float)
        leader.vel = np.array([10.0, 0.0, 0.0], dtype=float)

        # Advance 1 step: heading establishes as 0.0 (along +X)
        self.sim.step()
        self.assertAlmostEqual(brain.formation_heading, 0.0, places=2)

        # Now place leader within 4.0m of target (distance < 8.0m FREEZE_DIST)
        leader.pos = np.array([96.0, 0.0, 20.0], dtype=float)
        # Simulate micro-reversal / overshoot: vel points backwards along -X
        leader.vel = np.array([-2.0, 0.0, 0.0], dtype=float)

        self.sim.step()
        # Formation heading MUST remain frozen at 0.0 and NOT flip 180 degrees
        self.assertAlmostEqual(brain.formation_heading, 0.0, places=2)

    def test_echelon_heading_freeze_at_low_speed(self):
        """Verify formation heading freezes when leader speed drops below 1.5 m/s."""
        brain: LeaderFollowerBrain = self.sim.brain
        leader = self.sim.leader

        # Position leader far from target (50m) moving at cruise speed
        self.sim.target_pos = np.array([100.0, 0.0, 20.0], dtype=float)
        leader.pos = np.array([50.0, 0.0, 20.0], dtype=float)
        leader.vel = np.array([10.0, 0.0, 0.0], dtype=float)
        self.sim.step()
        self.assertAlmostEqual(brain.formation_heading, 0.0, places=2)

        # Now leader decelerates to 0.5 m/s (below FREEZE_SPEED 1.5 m/s) and veers +Y
        leader.vel = np.array([0.0, 0.5, 0.0], dtype=float)
        self.sim.step()
        # Formation heading must remain frozen at 0.0
        self.assertAlmostEqual(brain.formation_heading, 0.0, places=2)

    def test_echelon_heading_slew_rate_limit(self):
        """Verify heading change per step is strictly bounded by MAX_SLEW_RATE (45 deg/s)."""
        brain: LeaderFollowerBrain = self.sim.brain
        leader = self.sim.leader

        self.sim.target_pos = np.array([100.0, 0.0, 20.0], dtype=float)
        leader.pos = np.array([20.0, 0.0, 20.0], dtype=float)
        leader.vel = np.array([10.0, 0.0, 0.0], dtype=float)
        self.sim.step()
        self.assertAlmostEqual(brain.formation_heading, 0.0, places=2)

        # Leader suddenly banks 90 degrees (+Y) at full cruise speed
        leader.vel = np.array([0.0, 10.0, 0.0], dtype=float)
        dt = DT
        self.sim.step()

        # Max allowed delta is (pi / 4.0) * dt + slight float tolerance
        max_allowed_delta = (np.pi / 4.0) * dt + 1e-4
        delta_heading = abs(brain.formation_heading - 0.0)
        self.assertLessEqual(delta_heading, max_allowed_delta)

    def test_steer_toward_arrival_deadband(self):
        """Verify 0.25m arrival deadband outputs pure damping to prevent micro-hunting."""
        pos = np.array([10.0, 10.0, 20.0], dtype=float)
        vel = np.array([1.0, -0.5, 0.0], dtype=float)

        # Target within 0.1m (< 0.25m deadband)
        target = np.array([10.05, 10.05, 20.0], dtype=float)
        acc = LeaderFollowerBrain._steer_toward(pos, vel, target, max_speed=12.0)

        # Output must be pure velocity damping (-vel * 3.0)
        expected_acc = -vel * 3.0
        np.testing.assert_allclose(acc, expected_acc)


if __name__ == "__main__":
    unittest.main()
