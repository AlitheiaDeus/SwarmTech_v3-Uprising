"""
Unit tests for TKT-16: Tether Failsafes: Breakaway Dive, Cable Jettison, Escort Rejoin, & Doctrine A.
"""

import unittest
import numpy as np

from backend.sim_core import SwarmSimulator, ZONE4_CLASH, V_CRUISE


class TestTetherFailsafes(unittest.TestCase):
    """Test suite validating emergency breakaway dive, umbilical jettison,
    dynamic 2IC succession, and Doctrine A lost-link loiter & automated RTB."""

    def test_tether_severance_no_crash_and_jettison_timer(self):
        """Severing tether must not crash Drone 0; it enters BREAKAWAY_JETTISON and promotes 2IC."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_formation("ECHELON_ESCORT")

        # Cruise forward for 1.0s to establish flight
        for _ in range(60):
            sim.step()

        d0 = sim.drones[0]
        d1 = sim.drones[1]
        self.assertTrue(d0.is_leader)
        self.assertFalse(d0.crashed)
        self.assertEqual(sim.get_2ic().id, 1)

        # Sever tether
        sim.cut_tether()

        self.assertTrue(sim.tether_severed)
        self.assertFalse(d0.is_leader)
        self.assertFalse(d0.crashed, "Drone 0 must not crash upon tether severance!")
        self.assertEqual(d0.status, "BREAKAWAY_JETTISON")
        self.assertEqual(d0.jettison_timer, 1.5)
        self.assertFalse(d0.tether_jettisoned)

        # Dynamic 2IC (Drone 1) promoted to leader
        self.assertTrue(d1.is_leader)
        self.assertEqual(d1.status, "PROMOTED_LEADER")

        # Doctrine A 30s loiter initiated
        self.assertEqual(sim.lost_link_loiter_timer, 30.0)
        self.assertFalse(sim.swarm_rtb_active)

    def test_dynamic_2ic_selection(self):
        """Dynamic 2IC election correctly skips dead or salvaged drones."""
        sim = SwarmSimulator(drone_count=6)
        # Kill Drone 1 prior to cut
        sim.kill_drone(1)
        self.assertTrue(sim.drones[1].crashed)

        # 2IC should now be Drone 2
        two_ic = sim.get_2ic()
        self.assertIsNotNone(two_ic)
        self.assertEqual(two_ic.id, 2)

        # Cut tether -> Drone 2 should be promoted
        sim.cut_tether()
        self.assertTrue(sim.drones[2].is_leader)
        self.assertEqual(sim.drones[2].status, "PROMOTED_LEADER")

    def test_breakaway_dive_and_cable_purge(self):
        """Drone 0 dives vertically and purges cable after 1.5 seconds."""
        sim = SwarmSimulator(drone_count=6)
        for _ in range(60):
            sim.step()

        d0 = sim.drones[0]
        d1 = sim.drones[1]

        sim.cut_tether()

        # Step 1.33s (80 ticks) into the 1.5s jettison dive
        for _ in range(80):
            sim.step()

        # Drone 0 should be in breakaway dive below leader
        self.assertLess(d0.pos[2], d1.pos[2] - 2.0, "Drone 0 should dive below leader during jettison!")
        self.assertEqual(d0.status, "BREAKAWAY_JETTISON")
        self.assertGreater(d0.jettison_timer, 0.0)

        # Step remaining time to reach t = 1.67s post-severance (20 more ticks)
        for _ in range(20):
            sim.step()

        self.assertEqual(d0.jettison_timer, 0.0)
        self.assertTrue(d0.tether_jettisoned)
        self.assertEqual(d0.status, "ACTIVE")
        self.assertFalse(d0.crashed)

    def test_escort_rejoin_after_purge(self):
        """Unencumbered Drone 0 climbs back to cruise altitude and rejoins formation with zero clashes."""
        sim = SwarmSimulator(drone_count=6)
        for _ in range(60):
            sim.step()

        d0 = sim.drones[0]
        sim.cut_tether()

        min_dist = 999.0
        # Step through 4.0s (240 ticks)
        for _ in range(240):
            sim.step()
            living = [d for d in sim.drones if not d.crashed]
            for i in range(len(living)):
                for j in range(i + 1, len(living)):
                    dist = float(np.linalg.norm(living[i].pos - living[j].pos))
                    if dist < min_dist:
                        min_dist = dist

        # Drone 0 should have climbed back to formation cruise altitude (>= 20m)
        self.assertGreaterEqual(d0.pos[2], 20.0, "Drone 0 should climb back to cruise altitude upon rejoin!")
        self.assertEqual(d0.status, "ACTIVE")
        # Ensure zero prop-clashes
        self.assertGreater(min_dist, ZONE4_CLASH, "Pairwise distance must exceed Zone 4 clash envelope!")

    def test_doctrine_a_loiter_grace_period_and_automated_rtb(self):
        """Fleet holds target for 30s grace period; if no reconnection, triggers automated SWARM_RTB."""
        sim = SwarmSimulator(drone_count=6)
        target = [70.0, 50.0, 30.0]
        sim.set_target(target)

        for _ in range(60):
            sim.step()

        sim.cut_tether()
        self.assertEqual(sim.lost_link_loiter_timer, 30.0)
        self.assertFalse(sim.swarm_rtb_active)

        # Step 10.0 seconds (600 ticks)
        for _ in range(600):
            sim.step()

        self.assertAlmostEqual(sim.lost_link_loiter_timer, 20.0, delta=0.2)
        self.assertFalse(sim.swarm_rtb_active)
        np.testing.assert_allclose(sim.target_pos, target)

        # Step remaining 21.0 seconds (1260 ticks) to exceed 30.0s grace period
        for _ in range(1260):
            sim.step()

        self.assertEqual(sim.lost_link_loiter_timer, 0.0)
        self.assertTrue(sim.swarm_rtb_active)
        # Target must automatically be redirected to Base [0, 0, 25.0]
        np.testing.assert_allclose(sim.target_pos, [0.0, 0.0, 25.0])

    def test_tether_failsafe_policy_rtb_override(self):
        """When drone policy is set to RTB, drone returns to base after cable purge."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_target([80.0, 80.0, 30.0])
        for _ in range(60):
            sim.step()

        d0 = sim.drones[0]
        d0.tether_failsafe_policy = "RTB"

        sim.cut_tether()

        # Step 3.5s (210 ticks): past 1.5s purge and onto RTB transit
        for _ in range(210):
            sim.step()

        self.assertTrue(d0.tether_jettisoned)
        # Leader steers toward [80, 80, 30] while Drone 0 steers back toward [0, 0, 25]
        d1 = sim.drones[1]
        self.assertGreater(d1.pos[0], 10.0)
        # Drone 0's X position should be behind leader (flying back to base)
        self.assertLess(d0.pos[0], d1.pos[0])


if __name__ == "__main__":
    unittest.main()
