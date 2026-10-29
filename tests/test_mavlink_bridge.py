"""
Automated Unit Tests for SwarmTech3_UpRising Network Bridge (mavlink_bridge.py)
Tests UDP JSON telemetry broadcast (port 9000), command listener (port 9001),
and simulated MAVLink v2 loopback (port 14550).
"""

import json
import socket
import unittest
import time
import numpy as np

from SIM.backend.sitl_engine import SwarmSimulator
from SIM.backend.mavlink_bridge import MavlinkBridge, HAVE_PYMAVLINK

if HAVE_PYMAVLINK:
    from pymavlink.dialects.v20 import common as mavlink2


class TestMavlinkBridge(unittest.TestCase):
    """Test suite verifying UDP and MAVLink network interfaces."""

    def setUp(self):
        self.sim = SwarmSimulator(drone_count=5, enable_packet_loss=False, seed=10)
        # Use custom high test ports to avoid any environment port collisions
        self.host = "127.0.0.1"
        self.tx_port = 19000
        self.rx_port = 19001
        self.mav_port = 19550

        self.bridge = MavlinkBridge(
            sim=self.sim,
            host=self.host,
            tx_port=self.tx_port,
            rx_port=self.rx_port,
            mavlink_port=self.mav_port,
            enable_mavlink=True,
        )

    def tearDown(self):
        self.bridge.close()

    def test_telemetry_broadcast(self):
        """Verify telemetry snapshot JSON broadcast over UDP."""
        # Create client listener socket
        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.tx_port))
        listener.settimeout(1.0)

        try:
            bytes_sent = self.bridge.broadcast_telemetry()
            self.assertGreater(bytes_sent, 0)

            data, addr = listener.recvfrom(65535)
            payload = json.loads(data.decode("utf-8"))

            self.assertIn("sim_time", payload)
            self.assertIn("formation", payload)
            self.assertIn("drones", payload)
            self.assertEqual(len(payload["drones"]), 5)
        finally:
            listener.close()

    def test_command_ingestion(self):
        """Verify receiving and executing external JSON commands via UDP."""
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            # 1. Test SET_TARGET
            cmd_target = {"cmd": "SET_TARGET", "pos": [120.0, 80.0, 45.0]}
            sender.sendto(json.dumps(cmd_target).encode("utf-8"), (self.host, self.rx_port))

            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0]["status"], "OK")
            np.testing.assert_allclose(self.sim.target_pos, [120.0, 80.0, 45.0])

            # 2. Test SET_FORMATION
            cmd_formation = {"cmd": "SET_FORMATION", "mode": "ORBIT_VORTEX"}
            sender.sendto(json.dumps(cmd_formation).encode("utf-8"), (self.host, self.rx_port))

            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(self.sim.formation_mode, "ORBIT_VORTEX")

            # 3. Test CUT_TETHER
            cmd_cut = {"cmd": "CUT_TETHER"}
            sender.sendto(json.dumps(cmd_cut).encode("utf-8"), (self.host, self.rx_port))

            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertTrue(self.sim.tether_severed)

            # 4. Test KILL_LEADER
            old_leader_id = self.sim.leader.id
            cmd_kill = {"cmd": "KILL_LEADER"}
            sender.sendto(json.dumps(cmd_kill).encode("utf-8"), (self.host, self.rx_port))

            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertTrue(self.sim.drones[old_leader_id].crashed)
            self.assertNotEqual(self.sim.leader.id, old_leader_id)

            # 5b. Test CLEAR_WRECKAGE
            self.sim.kill_drone(1)
            cmd_clear = {"cmd": "CLEAR_WRECKAGE"}
            sender.sendto(json.dumps(cmd_clear).encode("utf-8"), (self.host, self.rx_port))

            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0]["status"], "OK")
            self.assertIn(1, responses[0]["purged_ids"])
            self.assertFalse(any(d.id == 1 for d in self.sim.drones))

            # 5c. Test TOGGLE_PATROL
            cmd_patrol = {"cmd": "TOGGLE_PATROL"}
            sender.sendto(json.dumps(cmd_patrol).encode("utf-8"), (self.host, self.rx_port))

            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0]["status"], "OK")
            self.assertTrue(responses[0]["patrol_active"])
            self.assertTrue(self.sim.patrol_active)

            # 5d. Test RTB_HOLD (TKT-19)
            cmd_rtb = {"cmd": "RTB_HOLD"}
            sender.sendto(json.dumps(cmd_rtb).encode("utf-8"), (self.host, self.rx_port))
            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0]["status"], "OK")
            self.assertTrue(self.sim.overhead_marshal_active)

            # 5e. Test LAND_SWARM (TKT-19)
            cmd_land = {"cmd": "LAND_SWARM"}
            sender.sendto(json.dumps(cmd_land).encode("utf-8"), (self.host, self.rx_port))
            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0]["status"], "OK")
            self.assertEqual(self.sim.recovery_stage, "LANDING")

        finally:
            sender.close()

    def test_mavlink_loopback_emission(self):
        """Verify binary MAVLink v2 packets emitted on loopback port."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.mav_port))
        listener.settimeout(1.0)

        try:
            # Emit MAVLink packet
            bytes_sent = self.bridge.emit_mavlink_loopback()
            self.assertGreater(bytes_sent, 0)

            data, addr = listener.recvfrom(2048)
            # MAVLink v2 magic byte is 0xFD
            self.assertEqual(data[0], 0xFD)

            # Parse with pymavlink decoder
            mav = mavlink2.MAVLink(None)
            msg = mav.decode(bytearray(data))
            # Should be HEARTBEAT (ID 0)
            self.assertEqual(msg.get_msgId(), 0)
            self.assertEqual(msg.type, mavlink2.MAV_TYPE_QUADROTOR)

        finally:
            listener.close()

    def test_unified_step(self):
        """Verify the unified bridge.step() method processes all streams."""
        res = self.bridge.step()
        self.assertIn("commands_processed", res)
        self.assertIn("telemetry_bytes_sent", res)
        self.assertIn("mavlink_bytes_sent", res)
        self.assertGreater(res["telemetry_bytes_sent"], 0)


if __name__ == "__main__":
    unittest.main()
