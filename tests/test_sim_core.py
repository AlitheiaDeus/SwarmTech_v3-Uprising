"""
Automated Unit Tests for SwarmTech3_UpRising Simulation Core (sim_core.py)
Tests 4-Zone physics, Zone 3 momentum damping, tether dynamics & snap,
10 Hz Baud & LKP recovery, autonomic succession, and formations.
"""

import unittest
import numpy as np
import json

from backend.sim_core import (
    SwarmSimulator,
    Drone,
    DT,
    ZONE4_CLASH,
    ZONE3_OUTER,
    ZONE2_OUTER,
    RADIO_RANGE,
    LINK_TIMEOUT,
    TETHER_SNAP_DIST,
    TETHER_AVOID_DIST,
    TETHER_MAX_LEN,
    TIER_LEADER,
    TIER_ESCORT,
)


class TestSimCore(unittest.TestCase):
    """Test suite verifying physics, safety zones, tether, comms, and succession."""

    def setUp(self):
        self.sim = SwarmSimulator(drone_count=6, enable_packet_loss=False, seed=42)

    def test_initialization(self):
        """Verify initial fleet composition and states."""
        self.assertEqual(len(self.sim.drones), 6)
        leader = self.sim.leader
        self.assertIsNotNone(leader)
        self.assertEqual(leader.id, 0)
        self.assertTrue(leader.is_leader)
        self.assertEqual(leader.tier, TIER_LEADER)
        self.assertFalse(self.sim.tether_severed)

        for drone in self.sim.drones[1:]:
            self.assertFalse(drone.is_leader)
            self.assertEqual(drone.tier, TIER_ESCORT)
            self.assertFalse(drone.crashed)
            self.assertFalse(drone.lost_link)

    def test_zone4_prop_clash(self):
        """Zone 4 (d <= 1.5m): Prop clash causes motor cut and ballistic crash."""
        d1 = self.sim.drones[1]
        d2 = self.sim.drones[2]

        # Place them at 1.2m separation (< 1.5m)
        d1.pos = np.array([10.0, 10.0, 20.0], dtype=float)
        d2.pos = np.array([10.0, 11.2, 20.0], dtype=float)

        self.sim.step()

        self.assertTrue(d1.crashed)
        self.assertTrue(d2.crashed)
        self.assertEqual(d1.status, "CRASHED")
        self.assertEqual(d2.status, "CRASHED")

    def test_zone3_relative_velocity_damping(self):
        """Zone 3 (1.5m < d <= 3.5m): Relative velocity damping prevents penetration."""
        d1 = self.sim.drones[1]
        d2 = self.sim.drones[2]

        # Position at 2.5m separation in Zone 3
        d1.pos = np.array([20.0, 0.0, 20.0], dtype=float)
        d2.pos = np.array([22.5, 0.0, 20.0], dtype=float)

        # Head-on closing velocities
        d1.vel = np.array([5.0, 0.0, 0.0], dtype=float)
        d2.vel = np.array([-5.0, 0.0, 0.0], dtype=float)

        avoidance = self.sim._compute_4zone_avoidance()
        acc_d1 = avoidance[d1.id]
        acc_d2 = avoidance[d2.id]

        # d1 should be strongly accelerated backward (-X)
        self.assertLess(acc_d1[0], -10.0)
        # d2 should be strongly accelerated forward (+X)
        self.assertGreater(acc_d2[0], 10.0)

        # Step forward 5 frames: verify relative velocity damping brakes closing speed
        initial_closing_speed = d1.vel[0] - d2.vel[0]  # 10.0 m/s
        for _ in range(5):
            self.sim.step()

        final_closing_speed = d1.vel[0] - d2.vel[0]
        self.assertLess(final_closing_speed, initial_closing_speed)
        # Ensure they did not clash
        self.assertFalse(d1.crashed)
        self.assertFalse(d2.crashed)

    def test_zone2_separation(self):
        """Zone 2 (3.5m < d <= 8.0m): Smooth linear separation repulsion."""
        d1 = self.sim.drones[1]
        d2 = self.sim.drones[2]

        # Separation = 5.0m (in Zone 2)
        d1.pos = np.array([30.0, 0.0, 20.0], dtype=float)
        d2.pos = np.array([35.0, 0.0, 20.0], dtype=float)
        d1.vel = np.zeros(3, dtype=float)
        d2.vel = np.zeros(3, dtype=float)

        avoidance = self.sim._compute_4zone_avoidance()
        # d1 should experience negative X push (away from d2)
        self.assertLess(avoidance[d1.id][0], 0.0)
        # d2 should experience positive X push (away from d1)
        self.assertGreater(avoidance[d2.id][0], 0.0)

    def test_zone1_nominal_clear(self):
        """Zone 1 (d > 8.0m): Nominal clear flight, zero repulsion force."""
        d1 = self.sim.drones[1]
        d2 = self.sim.drones[2]

        d1.pos = np.array([0.0, 0.0, 20.0], dtype=float)
        d2.pos = np.array([20.0, 0.0, 20.0], dtype=float)  # 20m > 8.0m

        avoidance = self.sim._compute_4zone_avoidance()
        np.testing.assert_allclose(avoidance[d1.id], [0.0, 0.0, 0.0], atol=1e-5)
        np.testing.assert_allclose(avoidance[d2.id], [0.0, 0.0, 0.0], atol=1e-5)

    def test_tether_avoidance(self):
        """Followers inside 3.0m tether envelope experience lateral repulsion."""
        leader = self.sim.leader
        leader.pos = np.array([0.0, 0.0, 50.0], dtype=float)
        # Tether segment is [0, 0, 1.5] to [0, 0, 50.0]

        follower = self.sim.drones[1]
        follower.pos = np.array([2.0, 0.0, 25.0], dtype=float)  # d_perp = 2.0m < 3.0m

        tether_accs = self.sim._compute_tether_avoidance()
        acc = tether_accs[follower.id]

        # Should push radially away in +X
        self.assertGreater(acc[0], 0.0)
        self.assertAlmostEqual(acc[1], 0.0, places=3)

    def test_tether_collision_snap_and_succession(self):
        """Follower penetrating d_perp <= 1.0m snaps tether and triggers succession."""
        leader = self.sim.leader
        old_leader_id = leader.id
        leader.pos = np.array([0.0, 0.0, 40.0], dtype=float)

        # Place follower right next to tether cable (0.5m away)
        follower = self.sim.drones[1]
        follower.pos = np.array([0.5, 0.0, 20.0], dtype=float)

        self.sim.step()

        self.assertTrue(self.sim.tether_severed)
        self.assertTrue(self.sim.drones[old_leader_id].tether_lost)
        # Autonomic succession should have elected a new leader
        new_leader = self.sim.leader
        self.assertIsNotNone(new_leader)
        self.assertNotEqual(new_leader.id, old_leader_id)
        self.assertTrue(new_leader.is_leader)

    def test_tether_tensile_snap_max_length(self):
        """Moving leader beyond 300m snaps cable under tensile limit."""
        leader = self.sim.leader
        leader.pos = np.array([0.0, 0.0, 310.0], dtype=float)  # > 300m

        self.sim.step()
        self.assertTrue(self.sim.tether_severed)

    def test_radio_baud_and_lkp_recovery(self):
        """10 Hz Baud & LKP: Follower loses link >120m, navigates to LKP, recovers on re-entry."""
        leader = self.sim.leader
        follower = self.sim.drones[1]

        # 1. Start within radio range (at 20m)
        leader.pos = np.array([0.0, 0.0, 20.0], dtype=float)
        follower.pos = np.array([10.0, 0.0, 20.0], dtype=float)

        # Run for 0.2s (12 frames) to receive at least one heartbeat
        for _ in range(12):
            self.sim.step()

        self.assertFalse(follower.lost_link)
        np.testing.assert_allclose(follower.lkp, leader.pos, atol=1.0)

        # 2. Teleport leader out of range (150m > 120m)
        recorded_lkp = np.copy(follower.lkp)
        leader.pos = np.array([150.0, 0.0, 20.0], dtype=float)

        # Advance 1.2 seconds (> 1.0s LINK_TIMEOUT = 72 ticks at 60 FPS)
        for _ in range(75):
            self.sim.step()

        self.assertTrue(follower.lost_link)
        self.assertEqual(follower.status, "LOST_LINK")

        # 3. Follower should steer toward its LKP
        # Distance to recorded LKP should decrease or remain near LKP
        dist_to_lkp = np.linalg.norm(follower.pos - recorded_lkp)
        self.assertLess(dist_to_lkp, 25.0)

        # 4. Bring leader back within range (< 120m)
        leader.pos = np.array([30.0, 0.0, 20.0], dtype=float)
        for _ in range(12):
            self.sim.step()

        self.assertFalse(follower.lost_link)
        self.assertEqual(follower.status, "ACTIVE")

    def test_autonomic_succession_leader_kill(self):
        """Killing the active leader promotes the highest-scoring follower."""
        # Initial leader is Drone #0
        self.assertEqual(self.sim.leader.id, 0)

        # Succession score = Tier * 1000 - ID
        # Living followers are #1, #2, #3, #4, #5 with Tier 2
        # Drone #1 has highest score: (2000 - 1) = 1999
        self.sim.kill_leader()

        self.assertTrue(self.sim.drones[0].crashed)
        new_leader = self.sim.leader
        self.assertIsNotNone(new_leader)
        self.assertEqual(new_leader.id, 1)
        self.assertEqual(new_leader.status, "PROMOTED_LEADER")
        self.assertTrue(new_leader.is_leader)

    def test_multi_mode_formations(self):
        """Verify dynamic switching between all 4 flight formation modes."""
        modes = [
            "ECHELON_ESCORT",
            "NATURAL_FLOCK",
            "ORBIT_VORTEX",
            "PERIMETER_PATROL",
        ]
        for mode in modes:
            self.sim.set_formation(mode)
            self.assertEqual(self.sim.formation_mode, mode)
            # Step 10 ticks to ensure no numerical singularities or NaN
            for _ in range(10):
                self.sim.step()
                for d in self.sim.drones:
                    self.assertFalse(np.isnan(d.pos).any())
                    self.assertFalse(np.isnan(d.vel).any())
                    self.assertFalse(np.isnan(d.acc).any())

    def test_state_snapshot_serialization(self):
        """Verify get_state_snapshot() is JSON serializable and contains required fields."""
        self.sim.step()
        snapshot = self.sim.get_state_snapshot()

        # Check JSON serialization
        json_str = json.dumps(snapshot)
        self.assertIsInstance(json_str, str)

        # Check fields
        self.assertIn("sim_time", snapshot)
        self.assertIn("dt", snapshot)
        self.assertIn("formation", snapshot)
        self.assertIn("target", snapshot)
        self.assertIn("tether", snapshot)
        self.assertIn("drones", snapshot)
        self.assertEqual(len(snapshot["drones"]), 6)


if __name__ == "__main__":
    unittest.main()
