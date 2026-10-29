"""
Unit tests for TKT-28: Discrete Waypoint Mission Queue & Custom Route Deployment.
Validates discrete route upload, arrival advancement, wrap-around looping,
tether-severance abort hierarchy, manual interrupt, and leadership succession inheritance.
"""

import unittest
import numpy as np

from SIM.backend.sitl_engine import SwarmSimulator, ROUTE_PRESETS


class TestMissionRoutes(unittest.TestCase):
    """Test suite validating discrete waypoint mission queues and route deployment."""

    def test_route_preset_deployment_and_obc_upload(self):
        """Deploying a route preset uploads waypoints to leader OBC and activates route."""
        sim = SwarmSimulator(drone_count=6)
        res = sim.start_route_preset("BOX_PATROL")

        self.assertEqual(res.get("status"), "OK")
        self.assertEqual(res.get("name"), "BOX_PATROL")
        self.assertTrue(sim.route_active)
        self.assertEqual(len(sim.route_waypoints), 4)

        leader = sim.leader
        self.assertIsNotNone(leader)
        self.assertTrue(leader.mission_active)
        self.assertEqual(len(leader.mission_waypoints), 4)
        self.assertEqual(leader.current_wp_idx, 0)
        np.testing.assert_allclose(leader.acknowledged_target, ROUTE_PRESETS["BOX_PATROL"][0])

    def test_waypoint_arrival_advancement_and_loop(self):
        """Leader advances to next waypoint upon entering 2.5m acceptance sphere and loops."""
        sim = SwarmSimulator(drone_count=4)
        custom_wps = [
            [20.0, 0.0, 20.0],
            [30.0, 10.0, 22.0],
            [20.0, 20.0, 25.0],
        ]
        res = sim.deploy_mission_route(custom_wps, loop=True, name="TRIANGLE_LOOP")
        self.assertEqual(res.get("status"), "OK")

        leader = sim.leader
        self.assertEqual(leader.current_wp_idx, 0)
        np.testing.assert_allclose(leader.acknowledged_target, custom_wps[0])

        # Teleport leader within acceptance sphere of WP 0
        leader.pos = np.array([20.5, 0.2, 20.1], dtype=float)
        sim.step()

        # Should have advanced to WP 1
        self.assertEqual(leader.current_wp_idx, 1)
        np.testing.assert_allclose(leader.acknowledged_target, custom_wps[1])

        # Teleport leader within acceptance sphere of WP 1
        leader.pos = np.array([30.1, 9.9, 22.2], dtype=float)
        sim.step()

        # Should have advanced to WP 2
        self.assertEqual(leader.current_wp_idx, 2)
        np.testing.assert_allclose(leader.acknowledged_target, custom_wps[2])

        # Teleport leader within acceptance sphere of final WP 2 -> should wrap around to WP 0
        leader.pos = np.array([19.9, 20.1, 24.9], dtype=float)
        sim.step()

        self.assertEqual(leader.current_wp_idx, 0)
        np.testing.assert_allclose(leader.acknowledged_target, custom_wps[0])
        self.assertTrue(leader.mission_active)

    def test_non_looping_route_deactivates_on_final_waypoint(self):
        """A route with loop=False terminates mission_active upon arriving at the final waypoint."""
        sim = SwarmSimulator(drone_count=4)
        wps = [
            [15.0, 0.0, 20.0],
            [25.0, 0.0, 22.0],
        ]
        sim.deploy_mission_route(wps, loop=False, name="POINT_TO_POINT")
        leader = sim.leader

        # Reach WP 0
        leader.pos = np.array([15.1, 0.0, 20.0], dtype=float)
        sim.step()
        self.assertEqual(leader.current_wp_idx, 1)

        # Reach WP 1 (final)
        leader.pos = np.array([25.0, 0.1, 22.0], dtype=float)
        sim.step()

        self.assertFalse(leader.mission_active)

    def test_tether_severance_aborts_mission_route(self):
        """Severing tether in TETHERED_RELAY mode immediately aborts active route and engages Doctrine A."""
        sim = SwarmSimulator(drone_count=6)
        sim.start_route_preset("DIAMOND_SURVEY")
        self.assertTrue(sim.route_active)
        self.assertTrue(sim.drones[0].mission_active)

        # Sever tether
        sim.cut_tether()

        # Route must be aborted to prevent infinite battery drain
        self.assertFalse(sim.route_active)
        for d in sim.drones:
            self.assertFalse(d.mission_active)

        # Doctrine A 30s loiter initiated
        self.assertEqual(sim.lost_link_loiter_timer, 30.0)

    def test_manual_target_dispatch_interrupts_route(self):
        """Operator dispatching a single target coordinate cancels the active mission route."""
        sim = SwarmSimulator(drone_count=4)
        sim.start_route_preset("BOX_PATROL")
        self.assertTrue(sim.route_active)

        sim.set_target([70.0, 70.0, 30.0])
        self.assertFalse(sim.route_active)
        for d in sim.drones:
            self.assertFalse(d.mission_active)

    def test_comms_loss_uplink_drop(self):
        """Deploying a route when tether is severed or RF range > 120m drops the uplink packet."""
        sim = SwarmSimulator(drone_count=4)
        # Sever tether
        sim.cut_tether()

        res = sim.start_route_preset("BOX_PATROL")
        self.assertEqual(res.get("status"), "DROPPED")
        self.assertEqual(res.get("reason"), "TETHER_SEVERED")

        # In wireless mode out of range
        sim.set_comms_mode("LEADER_FOLLOWER")
        sim.leader.pos = np.array([200.0, 0.0, 30.0], dtype=float)  # > 120m
        res2 = sim.start_route_preset("BOX_PATROL")
        self.assertEqual(res2.get("status"), "OUT_OF_RANGE")

    def test_leader_succession_inherits_route_in_wireless_mode(self):
        """In wireless LEADER_FOLLOWER mode, promoted 2IC inherits mission route and waypoint index."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_comms_mode("LEADER_FOLLOWER")
        sim.start_route_preset("HEXAGON_SWEEP")

        d0 = sim.drones[0]
        d1 = sim.drones[1]
        self.assertTrue(d0.is_leader)
        self.assertTrue(d0.mission_active)
        d0.current_wp_idx = 2  # At WP 2

        # Simulate leader motor cut
        sim.kill_leader()

        # Drone 1 should now be promoted leader and hold the same mission state
        self.assertTrue(d1.is_leader)
        self.assertTrue(d1.mission_active)
        self.assertEqual(d1.current_wp_idx, 2)
        self.assertEqual(len(d1.mission_waypoints), len(ROUTE_PRESETS["HEXAGON_SWEEP"]))

    def test_gossip_consensus_route_progression(self):
        """In Gossip consensus mode, swarm centroid arrival advances the consensus waypoint."""
        sim = SwarmSimulator(drone_count=4)
        sim.set_brain("GOSSIP_CONSENSUS")
        custom_wps = [
            [20.0, 0.0, 20.0],
            [35.0, 20.0, 25.0],
        ]
        res = sim.deploy_mission_route(custom_wps, loop=True, name="GOSSIP_ROUTE")
        self.assertEqual(res.get("status"), "OK")
        self.assertEqual(sim.brain.current_route_wp_idx, 0)

        # Position drones around WP 0 with safe physical separation (> 3.5m) so centroid arrives at WP 0
        offsets = [
            np.array([2.0, 0.0, 0.0], dtype=float),
            np.array([-2.0, 0.0, 0.0], dtype=float),
            np.array([0.0, 2.0, 0.0], dtype=float),
            np.array([0.0, -2.0, 0.0], dtype=float),
        ]
        for d, off in zip(sim.drones, offsets):
            d.pos = np.array([20.0, 0.0, 20.0], dtype=float) + off

        sim.step()

        # Gossip brain should have advanced to WP 1
        self.assertEqual(sim.brain.current_route_wp_idx, 1)
        np.testing.assert_allclose(sim.get_acknowledged_target(), custom_wps[1])


if __name__ == "__main__":
    unittest.main()
