"""
SwarmTech3_UpRising: Decentralized Consensus Formations Test Suite (test_gossip_formations.py)
Verifies TKT-11:
- Structured decentralized swarm geometries in GossipConsensusBrain:
  Perimeter Sentry Shell, Orbit Vortex, Chevron Echelon, and Dispersed Lattice.
- Deterministic peer slot allocation without a central leader.
- Dynamic slot re-indexing on casualty / salvage.
- Zero clumping and zero Zone 4 prop-clash across all modes.
"""

import unittest
import math
import numpy as np
from SIM.backend.sitl_engine import SwarmSimulator, ZONE4_CLASH


class TestGossipFormations(unittest.TestCase):

    def test_gossip_perimeter_patrol_geometry(self):
        """Verify PERIMETER_PATROL arranges drones into an equidistant azimuthal ring (R=25m)."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_comms_mode("GOSSIP_CONSENSUS")
        sim.set_formation("PERIMETER_PATROL")
        target = np.array([0.0, 0.0, 25.0], dtype=float)
        sim.set_target(list(target))

        # Advance 300 steps (5.0s) for drones to arrive at perimeter stations
        for _ in range(300):
            sim.step()

        active = [d for d in sim.drones if not d.crashed]
        self.assertEqual(len(active), 6)

        # Verify each drone is approximately 25m from consensus target
        for d in active:
            dist_xy = float(np.linalg.norm(d.pos[:2] - target[:2]))
            self.assertAlmostEqual(dist_xy, 25.0, delta=2.0)

        # Verify pairwise separation: no clumping, all distances > 10m
        for i in range(len(active)):
            for j in range(i + 1, len(active)):
                dist_ij = float(np.linalg.norm(active[i].pos - active[j].pos))
                self.assertGreater(dist_ij, 10.0, f"Drones #{active[i].id} and #{active[j].id} clumped: {dist_ij:.2f}m")

    def test_gossip_orbit_vortex_geometry(self):
        """Verify ORBIT_VORTEX causes drones to rotate around target with altitude wave."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_comms_mode("GOSSIP_CONSENSUS")
        sim.set_formation("ORBIT_VORTEX")
        target = np.array([0.0, 0.0, 25.0], dtype=float)
        sim.set_target(list(target))

        # 240 steps (4.0s) to establish steady-state circular orbit
        for _ in range(240):
            sim.step()

        # Capture positions
        pos_t1 = [np.copy(d.pos) for d in sim.drones]

        # Step 60 more frames (1.0s)
        for _ in range(60):
            sim.step()

        pos_t2 = [np.copy(d.pos) for d in sim.drones]

        # Check angular rotation around target (~0.6 rad)
        for p1, p2 in zip(pos_t1, pos_t2):
            th1 = math.atan2(p1[1] - target[1], p1[0] - target[0])
            th2 = math.atan2(p2[1] - target[1], p2[0] - target[0])
            d_th = (th2 - th1 + math.pi) % (2.0 * math.pi) - math.pi
            self.assertGreater(d_th, 0.2, "Drone did not exhibit tangential vortex orbit motion")

    def test_gossip_echelon_escort_geometry(self):
        """Verify ECHELON_ESCORT forms a symmetrical V-shape around the consensus target."""
        sim = SwarmSimulator(drone_count=5)
        sim.set_comms_mode("GOSSIP_CONSENSUS")
        sim.set_formation("ECHELON_ESCORT")
        target = np.array([25.0, 0.0, 20.0], dtype=float)
        sim.set_target(list(target))

        for _ in range(300):
            sim.step()

        active = sorted([d for d in sim.drones if not d.crashed], key=lambda d: d.id)
        self.assertEqual(len(active), 5)

        # Apex drone (slot 0) should be closest to target (< 2.0m)
        apex_dist = float(np.linalg.norm(active[0].pos - target))
        self.assertLess(apex_dist, 2.0)

        # Followers (slots 1, 2) and (slots 3, 4) should be further back along heading
        for i in range(1, len(active)):
            dist_i = float(np.linalg.norm(active[i].pos - target))
            self.assertGreater(dist_i, 3.0)

        # Check pairwise distances to guarantee zero Zone 4 clashes
        for i in range(len(active)):
            for j in range(i + 1, len(active)):
                d_ij = float(np.linalg.norm(active[i].pos - active[j].pos))
                self.assertGreater(d_ij, ZONE4_CLASH)

    def test_gossip_natural_flock_lattice(self):
        """Verify NATURAL_FLOCK distributes drones across a dispersed grid lattice."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_comms_mode("GOSSIP_CONSENSUS")
        sim.set_formation("NATURAL_FLOCK")
        target = np.array([0.0, 0.0, 25.0], dtype=float)
        sim.set_target(list(target))

        for _ in range(300):
            sim.step()

        active = [d for d in sim.drones if not d.crashed]
        # Check pairwise distances: should be spaced by at least 5.0m (nominal grid 8m)
        for i in range(len(active)):
            for j in range(i + 1, len(active)):
                dist = float(np.linalg.norm(active[i].pos - active[j].pos))
                self.assertGreater(dist, 5.0, f"Lattice nodes clumped: {dist:.2f}m")

    def test_gossip_formation_command_switching(self):
        """Verify SET_FORMATION updates GossipConsensusBrain and snapshot telemetry."""
        sim = SwarmSimulator(drone_count=4)
        sim.set_comms_mode("GOSSIP_CONSENSUS")
        self.assertEqual(sim.brain.formation_mode, "ECHELON_ESCORT")

        # Test switching to PERIMETER_PATROL
        sim.set_formation("PERIMETER_PATROL")
        self.assertEqual(sim.brain.formation_mode, "PERIMETER_PATROL")
        snap = sim.get_state_snapshot()
        self.assertEqual(snap["formation"], "PERIMETER_PATROL")

        # Test switching via brain.handle_command
        res = sim.brain.handle_command({"cmd": "SET_FORMATION", "mode": "ORBIT_VORTEX"})
        self.assertIsNotNone(res)
        self.assertEqual(res.get("status"), "OK")
        self.assertEqual(sim.brain.formation_mode, "ORBIT_VORTEX")

    def test_gossip_casualty_reorganization(self):
        """Verify that when a drone crashes in Gossip mode, the remaining nodes re-index cleanly."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_comms_mode("GOSSIP_CONSENSUS")
        sim.set_formation("PERIMETER_PATROL")
        sim.set_target([30.0, 30.0, 25.0])

        for _ in range(120):
            sim.step()

        # Kill drone #2
        sim.kill_drone(2)
        self.assertTrue(sim.drones[2].crashed)

        # Step 120 more frames for reorganization
        for _ in range(120):
            sim.step()

        survivors = [d for d in sim.drones if not d.crashed]
        self.assertEqual(len(survivors), 5)

        # Check no prop clash among survivors
        for i in range(len(survivors)):
            for j in range(i + 1, len(survivors)):
                d_ij = float(np.linalg.norm(survivors[i].pos - survivors[j].pos))
                self.assertGreater(d_ij, ZONE4_CLASH)

    def test_gossip_overhead_marshal_compatibility(self):
        """Verify that RTB Overhead Marshal places gossip nodes into 20m holding ring at Base."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_comms_mode("GOSSIP_CONSENSUS")
        sim.set_target([50.0, 50.0, 25.0])

        for _ in range(60):
            sim.step()

        # Issue RTB hold
        sim.command_rtb_hold()
        self.assertTrue(sim.overhead_marshal_active)

        for _ in range(240):
            sim.step()

        active = [d for d in sim.drones if not d.crashed]
        for d in active:
            # Distance from Base (0, 0, 25m) should be ~20m
            d_base_xy = float(np.linalg.norm(d.pos[:2]))
            self.assertAlmostEqual(d_base_xy, 20.0, delta=4.0)
            self.assertAlmostEqual(d.pos[2], 25.0, delta=3.0)


if __name__ == "__main__":
    unittest.main()
