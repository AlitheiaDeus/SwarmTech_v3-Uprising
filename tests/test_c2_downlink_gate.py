"""
Unit tests for Telemetry-Gated C2 Downlink Handshake & Uplink Failure Hold (TKT-31).
Validates:
1. Downlink arrival handshake strictly decoupled from physical ground truth.
2. Link loss (tether cut, radio range exceeded) suppresses arrival pings.
3. Uplink failure state machine freezes Commanded Target beacon (zero space-time spam).
4. Graceful recovery upon comms restoration.
"""

import unittest
import numpy as np
from SIM.backend.sitl_engine import SwarmSimulator


class TestC2DownlinkGate(unittest.TestCase):
    def test_downlink_link_health_tethered(self):
        """In TETHERED_RELAY, cutting the umbilical immediately severs GCS downlink."""
        sim = SwarmSimulator(drone_count=4)
        target = np.array([30.0, 30.0, 20.0], dtype=float)
        sim.set_target(target)

        leader = sim.leader
        self.assertIsNotNone(leader)

        # Move leader to target: link healthy, arrival verified
        leader.pos = np.copy(target)
        leader.acknowledged_target = np.copy(target)
        self.assertTrue(sim.is_link_active())
        self.assertTrue(sim.is_target_reached())

        # Sever tether: umbilical cut -> link drops, arrival must be False even if drone is physically at target
        sim.cut_tether()
        self.assertTrue(sim.tether_severed)
        self.assertFalse(sim.is_link_active())
        self.assertFalse(sim.is_target_reached())

    def test_downlink_link_health_wireless_star(self):
        """In LEADER_FOLLOWER, radio range and link loss govern GCS downlink reception."""
        sim = SwarmSimulator(drone_count=4)
        sim.set_comms_mode("LEADER_FOLLOWER")
        target = np.array([40.0, 40.0, 25.0], dtype=float)
        sim.set_target(target)

        leader = sim.leader
        leader.pos = np.copy(target)
        leader.acknowledged_target = np.copy(target)
        self.assertTrue(sim.is_link_active())
        self.assertTrue(sim.is_target_reached())

        # Move leader out of radio range (> 120m)
        leader.pos = np.array([150.0, 150.0, 30.0], dtype=float)
        self.assertFalse(sim.is_link_active())
        self.assertFalse(sim.is_target_reached())

        # Return leader into radio range and at target
        leader.pos = np.copy(target)
        self.assertTrue(sim.is_link_active())
        self.assertTrue(sim.is_target_reached())

        # Mark leader as lost_link
        leader.lost_link = True
        self.assertFalse(sim.is_link_active())
        self.assertFalse(sim.is_target_reached())

    def test_uplink_failure_hold_freezes_commanded_target(self):
        """When umbilical is cut during RNG recon, GCS freezes Commanded Target in UPLINK_OFFLINE hold."""
        sim = SwarmSimulator(drone_count=4)
        leader = sim.leader
        target = np.copy(sim.get_acknowledged_target())
        leader.pos = np.copy(target)

        # Engage RNG recon
        sim.toggle_rng_recon(True)
        self.assertTrue(sim.rng_recon_active)
        self.assertTrue(sim.rng_dwelling)
        self.assertEqual(sim.rng_status, "DWELLING")
        self.assertFalse(sim.rng_paused)

        initial_commanded_target = np.copy(sim.target_pos)

        # Sever tether
        sim.cut_tether()
        self.assertTrue(sim.tether_severed)

        # Step 120 frames (2.0s) through dwell and beyond
        for _ in range(120):
            sim.step()

        # Commanded Target MUST be frozen: no crazy jumping across space and time
        np.testing.assert_allclose(sim.target_pos, initial_commanded_target)
        self.assertEqual(sim.rng_status, "UPLINK_OFFLINE")
        self.assertTrue(sim.rng_paused)

        # Station count must not advance while offline
        self.assertEqual(sim.rng_station_count, 1)

    def test_uplink_failure_graceful_recovery(self):
        """When link is restored after severance hold, GCS unpauses and resumes normal progression."""
        sim = SwarmSimulator(drone_count=4)
        leader = sim.leader
        target = np.copy(sim.get_acknowledged_target())
        leader.pos = np.copy(target)

        sim.toggle_rng_recon(True)
        self.assertTrue(sim.rng_dwelling)

        initial_target = np.copy(sim.target_pos)

        # Sever tether and step into offline hold
        sim.cut_tether()
        for _ in range(30):
            sim.step()

        self.assertEqual(sim.rng_status, "UPLINK_OFFLINE")
        self.assertTrue(sim.rng_paused)
        np.testing.assert_allclose(sim.target_pos, initial_target)

        # Operator overrides/cancels RTB to normalize to LEADER_FOLLOWER wireless
        sim.cancel_rtb()
        self.assertEqual(sim.comms_mode, "LEADER_FOLLOWER")
        self.assertTrue(sim.is_link_active())

        # Position leader at station and step
        leader.pos = np.copy(initial_target)
        leader.acknowledged_target = np.copy(initial_target)

        for _ in range(10):
            sim.step()

        # Should recover from UPLINK_OFFLINE
        self.assertFalse(sim.rng_paused)
        self.assertIn(sim.rng_status, ("TRACKING", "DWELLING"))

    def test_state_snapshot_includes_tkt31_fields(self):
        """Snapshot correctly serializes rng status and paused state."""
        sim = SwarmSimulator(drone_count=4)
        sim.toggle_rng_recon(True)
        snapshot = sim.get_state_snapshot()
        rng = snapshot["rng_recon"]
        self.assertIn("status", rng)
        self.assertIn("paused", rng)
        self.assertEqual(rng["status"], sim.rng_status)
        self.assertEqual(rng["paused"], sim.rng_paused)


if __name__ == "__main__":
    unittest.main()
