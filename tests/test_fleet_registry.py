"""
Automated Unit Tests for Fleet Enrollment Registry & Hardware Identity (TKT-48)
Tests the 3-tier airframe identity hierarchy (64-bit silicon UID, tail numbers, sysid),
standby reserve allocation, casualty preservation, and MAVLink AUTOPILOT_VERSION (#148).
"""

import json
import socket
import unittest
import numpy as np

from SIM.backend.fleet_registry import FleetRegistry, AirframeRecord, CasualtyEntry
from SIM.backend.sitl_engine import SwarmSimulator
from SIM.backend.mavlink_bridge import MavlinkBridge, HAVE_PYMAVLINK, STANDARD_PARAMS

if HAVE_PYMAVLINK:
    from pymavlink.dialects.v20 import common as mavlink2


class TestFleetRegistry(unittest.TestCase):
    """Test suite for FleetRegistry and airframe hardware identity."""

    def test_airframe_identity_hierarchy(self):
        """Verify 3-tier identity: 64-bit silicon UID, alphanumeric tail numbers, and dynamic sysid."""
        registry = FleetRegistry(initial_count=8, reserve_count=16)

        # 1. Check total registered
        snapshot = registry.get_registry_snapshot()
        self.assertEqual(snapshot["active_count"], 8)
        self.assertEqual(snapshot["reserve_count"], 16)
        self.assertEqual(snapshot["casualty_count"], 0)

        # 2. Check initial active airframes
        for i in range(8):
            rec = registry.get_by_drone_id(i)
            self.assertIsNotNone(rec)
            self.assertEqual(rec.tail_number, f"ALITHEIA-V{i + 1:02d}")
            self.assertEqual(rec.sysid, i + 1)
            self.assertEqual(rec.pad_id, i)
            self.assertEqual(rec.status, "ACTIVE")

            # Check 64-bit hex UID format
            self.assertEqual(len(rec.hardware_uid), 16)
            expected_uid_int = FleetRegistry.BASE_UID_PREFIX + (i + 1)
            self.assertEqual(rec.hardware_uid, f"{expected_uid_int:016X}")
            self.assertEqual(rec.hardware_uid_int, expected_uid_int)

            # Check 18-byte uid2 array
            uid2 = rec.uid2_bytes
            self.assertEqual(len(uid2), 18)
            self.assertEqual(bytes(uid2[:8]), expected_uid_int.to_bytes(8, "big"))

    def test_standby_reserve_allocation(self):
        """Verify standby reserve inventory allocation on reinforcement launch."""
        registry = FleetRegistry(initial_count=8, reserve_count=4)

        # Allocate first reserve for drone ID 8
        rec1 = registry.allocate_reserve(drone_id=8, pad_id=0, sim_time=12.5)
        self.assertEqual(rec1.tail_number, "ALITHEIA-RES-01")
        self.assertEqual(rec1.drone_id, 8)
        self.assertEqual(rec1.sysid, 9)
        self.assertEqual(rec1.status, "ACTIVE")
        self.assertEqual(rec1.registered_at, 12.5)
        self.assertEqual(rec1.hardware_uid, f"{FleetRegistry.RESERVE_UID_PREFIX + 1:016X}")

        # Allocate second reserve for drone ID 9
        rec2 = registry.allocate_reserve(drone_id=9, pad_id=1, sim_time=15.0)
        self.assertEqual(rec2.tail_number, "ALITHEIA-RES-02")
        self.assertEqual(rec2.drone_id, 9)
        self.assertEqual(rec2.sysid, 10)
        self.assertEqual(rec2.status, "ACTIVE")

        # Check remaining reserves
        snapshot = registry.get_registry_snapshot()
        self.assertEqual(snapshot["active_count"], 10)
        self.assertEqual(snapshot["reserve_count"], 2)

    def test_reserve_exhaustion_dynamic_minting(self):
        """Verify dynamic minting of fresh reserve airframes when standby pool is exhausted."""
        registry = FleetRegistry(initial_count=2, reserve_count=1)

        # Consume only pre-allocated reserve
        rec1 = registry.allocate_reserve(drone_id=2)
        self.assertEqual(rec1.tail_number, "ALITHEIA-RES-01")

        # Exceed pool: fresh reserve minted dynamically
        rec2 = registry.allocate_reserve(drone_id=3)
        self.assertEqual(rec2.tail_number, "ALITHEIA-RES-02")
        self.assertEqual(rec2.status, "ACTIVE")

    def test_casualty_preservation_and_wreckage_clearance(self):
        """Verify casualty logs permanently preserve downed airframe UIDs and LKPs."""
        sim = SwarmSimulator(drone_count=5, enable_packet_loss=False, seed=42)

        # Verify initial airframes have hardware identities
        for d in sim.drones:
            self.assertTrue(d.hardware_uid.startswith("3842A10F9B20"))
            self.assertTrue(d.tail_number.startswith("ALITHEIA-V"))

        victim = sim.drones[2]
        victim_uid = victim.hardware_uid
        victim_tail = victim.tail_number
        victim_pos = list(victim.pos)

        # Kill drone 2
        sim.kill_drone(2)
        self.assertTrue(victim.crashed)

        # Verify casualty recorded in fleet registry
        rec = sim.fleet_registry.get_by_drone_id(2)
        self.assertEqual(rec.status, "CASUALTY")
        self.assertEqual(rec.hardware_uid, victim_uid)

        cas_records = sim.fleet_registry.get_casualty_records()
        self.assertEqual(len(cas_records), 1)
        self.assertEqual(cas_records[0].hardware_uid, victim_uid)
        self.assertEqual(cas_records[0].tail_number, victim_tail)
        self.assertEqual(cas_records[0].drone_id, 2)
        np.testing.assert_allclose(cas_records[0].lkp, victim_pos, atol=0.1)

        # Clear physical wreckage: casualty log MUST persist
        purged = sim.clear_crashed_wreckage()
        self.assertIn(2, purged)
        self.assertEqual(len(sim.drones), 4)

        # Permanent preservation invariant: casualty log still has victim record
        cas_after = sim.fleet_registry.get_casualty_records()
        self.assertEqual(len(cas_after), 1)
        self.assertEqual(cas_after[0].hardware_uid, victim_uid)

    def test_reinforcement_allocates_fresh_reserve_identity(self):
        """Verify scrambled reinforcements pull fresh reserve call-signs without colliding with casualties."""
        sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=42)

        # Crash drone 1
        sim.kill_drone(1)
        dead_rec = sim.fleet_registry.get_by_drone_id(1)
        self.assertEqual(dead_rec.status, "CASUALTY")
        dead_uid = dead_rec.hardware_uid

        # Launch reinforcement
        reinforcement = sim.launch_reinforcement()
        self.assertEqual(reinforcement.id, 4)
        self.assertEqual(reinforcement.tail_number, "ALITHEIA-RES-01")
        self.assertNotEqual(reinforcement.hardware_uid, dead_uid)
        self.assertTrue(reinforcement.hardware_uid.startswith("3842A10F9B20FF"))

        # Verify both casualty and reinforcement are tracked distinctly
        snapshot = sim.fleet_registry.get_registry_snapshot()
        self.assertEqual(snapshot["casualty_count"], 1)
        self.assertEqual(snapshot["active_count"], 4)  # 3 surviving + 1 reinforcement

    def test_reboot_airframe_preserves_hardware_identity(self):
        """Verify field rebooting a casualty retains its original hardware UID and tail number."""
        sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=42)

        d = sim.drones[3]
        orig_uid = d.hardware_uid
        orig_tail = d.tail_number

        sim.kill_drone(3)
        self.assertEqual(sim.fleet_registry.get_by_drone_id(3).status, "CASUALTY")

        sim.reboot_drone(3)
        self.assertFalse(d.crashed)
        self.assertEqual(d.hardware_uid, orig_uid)
        self.assertEqual(d.tail_number, orig_tail)
        self.assertEqual(sim.fleet_registry.get_by_drone_id(3).status, "ACTIVE")

    def test_telemetry_snapshot_serialization(self):
        """Verify state snapshot includes fleet registry and individual airframe identities."""
        sim = SwarmSimulator(drone_count=3, enable_packet_loss=False, seed=42)
        snapshot = sim.get_state_snapshot()

        self.assertIn("fleet_registry", snapshot)
        self.assertEqual(snapshot["fleet_registry"]["active_count"], 3)
        self.assertEqual(snapshot["fleet_registry"]["casualty_count"], 0)

        for d_dict in snapshot["drones"]:
            self.assertIn("hardware_uid", d_dict)
            self.assertIn("tail_number", d_dict)
            self.assertIn("sysid", d_dict)
            self.assertEqual(d_dict["sysid"], d_dict["id"] + 1)


@unittest.skipUnless(HAVE_PYMAVLINK, "pymavlink not installed")
class TestMavlinkAutopilotVersion(unittest.TestCase):
    """Test suite verifying MAVLink AUTOPILOT_VERSION (#148) and parameter server."""

    def setUp(self):
        self.sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=10)
        self.host = "127.0.0.1"
        self.tx_port = 20000
        self.rx_port = 20001
        self.mav_port = 20550

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

    def test_autopilot_version_capabilities_probe(self):
        """Verify responding to MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES (520) with AUTOPILOT_VERSION (#148)."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, 20551))
        sock.settimeout(1.0)

        try:
            client_mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=190)
            bridge_port = self.bridge.mavlink_sock.getsockname()[1]

            # Send MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES (cmd 520) to vehicle 2 (drone #1)
            msg_cmd = client_mav.command_long_encode(
                target_system=2,
                target_component=1,
                command=520,  # MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
                confirmation=0,
                param1=1.0,   # 1 = request autopilot version
                param2=0, param3=0, param4=0, param5=0, param6=0, param7=0,
            )
            sock.sendto(msg_cmd.pack(client_mav), (self.host, bridge_port))

            # Poll on bridge
            responses = self.bridge.poll_mavlink_commands()
            self.assertTrue(any(r.get("cmd") == "MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES" for r in responses))

            # Receive packets back on client socket
            received_autopilot_version = False
            received_ack = False

            while True:
                try:
                    data, _ = sock.recvfrom(4096)
                    parsed = client_mav.parse_buffer(bytearray(data))
                    if not parsed:
                        continue
                    for p in parsed:
                        if p.get_msgId() == mavlink2.MAVLINK_MSG_ID_COMMAND_ACK:
                            self.assertEqual(p.command, 520)
                            self.assertEqual(p.result, mavlink2.MAV_RESULT_ACCEPTED)
                            self.assertEqual(p.get_srcSystem(), 2)
                            received_ack = True
                        elif p.get_msgId() == mavlink2.MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                            self.assertEqual(p.get_srcSystem(), 2)
                            # Verify 64-bit hardware UID matches Drone #1 (sysid 2)
                            expected_uid_int = self.sim.drones[1].fleet_record.hardware_uid_int if hasattr(self.sim.drones[1], "fleet_record") else int(self.sim.drones[1].hardware_uid, 16)
                            self.assertEqual(p.uid, expected_uid_int)
                            # Verify 18-byte uid2 array
                            self.assertEqual(bytes(p.uid2[:8]), expected_uid_int.to_bytes(8, "big"))
                            # Verify capabilities bitmask
                            self.assertGreater(p.capabilities, 0)
                            self.assertEqual(p.flight_sw_version, 0x030C0000)
                            received_autopilot_version = True
                except socket.timeout:
                    break

            self.assertTrue(received_ack, "COMMAND_ACK (520) was not received")
            self.assertTrue(received_autopilot_version, "AUTOPILOT_VERSION (#148) was not received")

        finally:
            sock.close()

    def test_request_message_autopilot_version(self):
        """Verify responding to MAV_CMD_REQUEST_MESSAGE (512, param1=148) with AUTOPILOT_VERSION (#148)."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, 20552))
        sock.settimeout(1.0)

        try:
            client_mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=190)
            bridge_port = self.bridge.mavlink_sock.getsockname()[1]

            # Send MAV_CMD_REQUEST_MESSAGE (cmd 512) for msg 148 to vehicle 1
            msg_cmd = client_mav.command_long_encode(
                target_system=1,
                target_component=1,
                command=512,  # MAV_CMD_REQUEST_MESSAGE
                confirmation=0,
                param1=148.0, # MAVLINK_MSG_ID_AUTOPILOT_VERSION
                param2=0, param3=0, param4=0, param5=0, param6=0, param7=0,
            )
            sock.sendto(msg_cmd.pack(client_mav), (self.host, bridge_port))

            responses = self.bridge.poll_mavlink_commands()
            self.assertTrue(any(r.get("cmd") == "MAV_CMD_REQUEST_MESSAGE" and r.get("requested") == 148 for r in responses))

            received_autopilot_version = False
            while True:
                try:
                    data, _ = sock.recvfrom(4096)
                    parsed = client_mav.parse_buffer(bytearray(data))
                    if not parsed:
                        continue
                    for p in parsed:
                        if p.get_msgId() == mavlink2.MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                            self.assertEqual(p.get_srcSystem(), 1)
                            expected_uid_int = int(self.sim.drones[0].hardware_uid, 16)
                            self.assertEqual(p.uid, expected_uid_int)
                            received_autopilot_version = True
                except socket.timeout:
                    break

            self.assertTrue(received_autopilot_version, "AUTOPILOT_VERSION (#148) not received for cmd 512")

        finally:
            sock.close()

    def test_parameter_table_includes_speed_parameters(self):
        """Verify STANDARD_PARAMS includes pilot speed parameters for instant Mission Planner dismissal."""
        param_names = [p[0] for p in STANDARD_PARAMS]
        self.assertIn("SYSID_THISMAV", param_names)
        self.assertIn("FRAME_CLASS", param_names)
        self.assertIn("ARMING_CHECK", param_names)
        self.assertIn("PILOT_SPEED_UP", param_names)
        self.assertIn("PILOT_SPEED_DN", param_names)


if __name__ == "__main__":
    unittest.main()
