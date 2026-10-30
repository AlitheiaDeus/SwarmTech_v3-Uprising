# ==============================================================================
# SwarmTech3_UpRising: Autonomous Swarm RTB & Coordinated Soft-Landing Test Suite
# Tests:
#   1. Pad geometry (>= 2.5m spacing, > 3.0m tether cylinder clearance, Z=0.5m)
#   2. Stage 1: Commanded RTB (Hold) / Overhead Marshal sentry ring at Z=25m
#   3. Cancel RTB & resume mission pursuit
#   4. Stage 2: Commanded LAND_SWARM with ATC pad reservation & 2.0s stagger
#   5. 2-stage vertical deceleration sink (-1.5 m/s to 1.0m, -0.5 m/s to 0.5m)
#   6. Touchdown disarm (motors cut, rotors 0 RPM, immune to formation pull)
#   7. Individual drone landing and relaunch
#   8. Doctrine A automated RTB transition upon loiter expiration
# ==============================================================================

import unittest
import numpy as np
import math

from SIM.backend.sitl_engine import SwarmSimulator, DT, TIER_LEADER, TIER_ESCORT
from OBC.backend.drone_firmware import Drone


class TestAutonomousSwarmRTBLanding(unittest.TestCase):
    """Test suite for TKT-19 2-Stage Tactical Recovery & Soft-Landing."""

    def test_pad_geometry_and_clearance(self):
        """Verify pad layout: >= 2.5m separation, > 3.0m tether clearance, Z=0.5m."""
        pads = [SwarmSimulator.get_pad_position(i) for i in range(8)]

        # 1. Verify Z coordinate is ground contact height 0.5m
        for i, pad in enumerate(pads):
            self.assertAlmostEqual(pad[2], 0.5, places=2, msg=f"Pad {i} Z should be 0.5m")

        # 2. Verify all pads are outside the 3.0m tether clearance envelope
        anchor = np.array([0.0, 0.0, 0.0])
        for i, pad in enumerate(pads):
            dist_to_anchor = float(np.linalg.norm(pad - anchor))
            self.assertGreater(
                dist_to_anchor,
                3.0,
                msg=f"Pad {i} at {pad} must be outside 3.0m tether cylinder (was {dist_to_anchor:.2f}m)"
            )

        # 3. Verify pairwise pad separation is at least 2.5m (well above 1.5m Zone 4 clash)
        for i in range(len(pads)):
            for j in range(i + 1, len(pads)):
                dist = float(np.linalg.norm(pads[i] - pads[j]))
                self.assertGreaterEqual(
                    dist,
                    2.49,
                    msg=f"Pads {i} and {j} must have >= 2.5m separation (was {dist:.2f}m)"
                )

    def test_stage1_overhead_marshal(self):
        """Test Stage 1: Commanded RTB (Hold) puts fleet into overhead marshal holding pattern."""
        sim = SwarmSimulator(drone_count=4)
        sim.command_rtb_hold()

        self.assertTrue(sim.overhead_marshal_active)
        self.assertTrue(sim.swarm_rtb_active)
        self.assertEqual(sim.recovery_stage, "OVERHEAD_MARSHAL")
        self.assertFalse(sim.patrol_active)
        self.assertTrue(np.allclose(sim.target_pos, [0.0, 0.0, 25.0]))

        # Advance sim for 150 steps (2.5 seconds)
        for _ in range(150):
            sim.step(DT)

        # All drones must remain airborne, armed, altitude >= 20m, zero crashes
        for drone in sim.drones:
            self.assertFalse(drone.crashed, f"Drone {drone.id} should not crash in marshal")
            self.assertGreater(drone.rotor_rpm, 4000.0, "Rotors must remain armed in marshal")
            self.assertGreater(drone.pos[2], 18.0, "Drones must maintain high holding altitude")

        # Snapshot should report overhead_marshal_active and recovery_stage
        snap = sim.get_state_snapshot()
        self.assertTrue(snap["overhead_marshal_active"])
        self.assertEqual(snap["recovery_stage"], "OVERHEAD_MARSHAL")

    def test_cancel_rtb(self):
        """Test canceling RTB returns living drones to active mission cruise."""
        sim = SwarmSimulator(drone_count=3)
        sim.command_rtb_hold()
        self.assertTrue(sim.overhead_marshal_active)

        sim.cancel_rtb()
        self.assertFalse(sim.overhead_marshal_active)
        self.assertFalse(sim.swarm_rtb_active)
        self.assertEqual(sim.recovery_stage, "IDLE")

        for drone in sim.drones:
            self.assertEqual(drone.flight_state, "CRUISE")

    def test_target_dispatch_auto_cancels_marshal(self):
        """Dispatching a new mission target waypoint away from base anchor automatically disengages marshal."""
        sim = SwarmSimulator(drone_count=3)
        sim.command_rtb_hold()
        self.assertTrue(sim.overhead_marshal_active)

        # Dispatch custom target
        sim.set_target([80.0, 50.0, 35.0])
        self.assertFalse(sim.overhead_marshal_active)
        self.assertFalse(sim.swarm_rtb_active)

    def test_stage2_land_swarm_atc_stagger(self):
        """Test Stage 2: Commanded LAND_SWARM allocates pads and staggers descent by 2.0s."""
        sim = SwarmSimulator(drone_count=4)
        queued = sim.command_land_swarm()

        self.assertEqual(queued, [0, 1, 2, 3])
        self.assertEqual(sim.recovery_stage, "LANDING")
        self.assertFalse(sim.overhead_marshal_active)

        # Pad allocation must be distinct
        pad_indices = [d.landing_pad_idx for d in sim.drones]
        self.assertEqual(len(set(pad_indices)), 4, "Each drone must receive a unique recovery pad")

        # Stagger clearance interval must be 2.0s
        self.assertAlmostEqual(sim.drones[0].landing_wait_timer, 0.0)
        self.assertAlmostEqual(sim.drones[1].landing_wait_timer, 2.0)
        self.assertAlmostEqual(sim.drones[2].landing_wait_timer, 4.0)
        self.assertAlmostEqual(sim.drones[3].landing_wait_timer, 6.0)

        # Initial landing stages
        self.assertEqual(sim.drones[0].landing_stage, "APPROACH")
        self.assertEqual(sim.drones[1].landing_stage, "WAITING_CLEARANCE")

    def test_coordinated_soft_landing_full_simulation(self):
        """Full simulation of 4 drones returning to base and executing 2-stage soft landing."""
        sim = SwarmSimulator(drone_count=4)
        sim.command_land_swarm()

        # Step sim until all drones touch down and disarm (max 30 seconds = 1800 ticks)
        for _ in range(1800):
            sim.step(DT)
            if all(d.flight_state == "LANDED" for d in sim.drones):
                break

        # Verify all drones reached LANDED state safely
        for drone in sim.drones:
            self.assertEqual(drone.flight_state, "LANDED", f"Drone {drone.id} should be LANDED")
            self.assertEqual(drone.status, "LANDED")
            self.assertFalse(drone.crashed, f"Drone {drone.id} should not clash during recovery")
            self.assertAlmostEqual(drone.pos[2], 0.5, delta=0.05, msg="Should be settled at 0.5m on pad")
            self.assertAlmostEqual(float(np.linalg.norm(drone.vel)), 0.0, delta=0.05, msg="Velocity must be 0")
            self.assertAlmostEqual(drone.rotor_rpm, 0.0, delta=0.1, msg="Rotors must be powered down to 0 RPM")

            # Pad alignment check
            expected_pad = sim.get_pad_position(drone.landing_pad_idx)
            self.assertAlmostEqual(float(np.linalg.norm(drone.pos[:2] - expected_pad[:2])), 0.0, delta=0.1)

    def test_single_drone_landing_and_relaunch(self):
        """Test commanding a single escort to land, followed by vertical relaunch."""
        sim = SwarmSimulator(drone_count=3)

        # Command drone 1 to land
        success = sim.command_land_drone(1)
        self.assertTrue(success)
        self.assertEqual(sim.drones[1].flight_state, "DESCENDING")
        self.assertEqual(sim.drones[1].landing_pad_idx, 0)

        # Step until drone 1 lands
        for _ in range(1000):
            sim.step(DT)
            if sim.drones[1].flight_state == "LANDED":
                break

        self.assertEqual(sim.drones[1].flight_state, "LANDED")
        self.assertEqual(sim.drones[1].rotor_rpm, 0.0)

        # Other drones should still be airborne in cruise
        self.assertIn(sim.drones[0].flight_state, ("CRUISE", "OVERHEAD_HOLD"))
        self.assertIn(sim.drones[2].flight_state, ("CRUISE", "OVERHEAD_HOLD"))

        # Now relaunch drone 1
        relaunch_ok = sim.command_takeoff_drone(1)
        self.assertTrue(relaunch_ok)
        self.assertEqual(sim.drones[1].flight_state, "TAKEOFF")
        self.assertEqual(sim.drones[1].rotor_rpm, 5200.0)
        self.assertNotIn(0, sim.landing_pads, "Pad 0 should be freed after takeoff")

        # Step until drone 1 climbs to safe altitude (Z >= 20m)
        for _ in range(300):
            sim.step(DT)
            if sim.drones[1].pos[2] >= 20.0:
                break

        self.assertGreaterEqual(sim.drones[1].pos[2], 20.0)
        self.assertIn(sim.drones[1].flight_state, ("EN_ROUTE", "CRUISE"))

    def test_doctrine_a_loiter_timeout_triggers_overhead_marshal(self):
        """Fleet Doctrine A: Tether cut loiters for 30s, then automatically triggers Stage 1 Overhead Marshal."""
        sim = SwarmSimulator(drone_count=3)
        sim.cut_tether()
        self.assertTrue(sim.tether_severed)
        self.assertEqual(sim.lost_link_loiter_timer, 30.0)
        self.assertFalse(sim.overhead_marshal_active)

        # Simulate 30 seconds of loiter (1800 ticks)
        for _ in range(1805):
            sim.step(DT)

        self.assertTrue(sim.overhead_marshal_active)
        self.assertTrue(sim.swarm_rtb_active)
        self.assertEqual(sim.recovery_stage, "OVERHEAD_MARSHAL")
        self.assertTrue(np.allclose(sim.target_pos, [0.0, 0.0, 25.0]))

    def test_out_of_horizon_recovery_rejection_and_reinforcement_muster(self):
        """Verify FIX-04: GCS commands are dropped beyond 120m radio horizon, and reinforcements muster locally."""
        sim = SwarmSimulator(drone_count=3)
        sim.comms_mode = "GOSSIP_CONSENSUS"

        # Relocate all drones well beyond 120m radio horizon
        for drone in sim.drones:
            drone.pos = np.array([160.0, 0.0, 30.0], dtype=float)

        self.assertFalse(sim.is_link_active(), "Link must be reported inactive when fleet is at 160m (>120m)")

        # 1. RTB command must be dropped over airgap
        res_rtb = sim.command_rtb_hold()
        self.assertIsInstance(res_rtb, dict)
        self.assertEqual(res_rtb.get("status"), "DROPPED")
        self.assertEqual(res_rtb.get("reason"), "OUT_OF_RANGE")
        self.assertFalse(sim.overhead_marshal_active, "Overhead marshal must NOT engage when command dropped")

        # 2. LAND_SWARM command must be dropped over airgap
        res_land = sim.command_land_swarm()
        self.assertIsInstance(res_land, dict)
        self.assertEqual(res_land.get("status"), "DROPPED")
        self.assertEqual(res_land.get("reason"), "OUT_OF_RANGE")

        # 3. Newly launched reinforcement drone cannot peek at distant fleet coordinates
        new_drone = sim.launch_reinforcement()
        self.assertIsNotNone(new_drone)

        # Reinforced drone must start in LOST_LINK with base muster coordinates, not leader's 160m pos
        self.assertTrue(new_drone.lost_link)
        self.assertEqual(new_drone.status, "LOST_LINK")
        self.assertTrue(
            np.allclose(new_drone.lkp, [0.0, 0.0, 20.0]),
            f"New drone must muster at local base [0, 0, 20], got {new_drone.lkp}"
        )


if __name__ == "__main__":
    unittest.main()
