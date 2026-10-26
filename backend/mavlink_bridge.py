"""
SwarmTech3_UpRising: UDP & MAVLink Network Bridge (mavlink_bridge.py)
Transmits swarm telemetry JSON on UDP port 9000,
receives operator command JSON on UDP port 9001,
and emits simulated MAVLink v2 binary telemetry on UDP port 14550.
"""

from __future__ import annotations
import json
import math
import socket
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

try:
    from pymavlink.dialects.v20 import common as mavlink2
    HAVE_PYMAVLINK = True
except ImportError:
    HAVE_PYMAVLINK = False

from backend.sim_core import SwarmSimulator, Drone


# ==============================================================================
# Network Port Constants
# ==============================================================================
DEFAULT_HOST: str = "127.0.0.1"
DEFAULT_TX_PORT: int = 9000        # UDP Swarm state JSON broadcast
DEFAULT_RX_PORT: int = 9001        # UDP External command listener
DEFAULT_MAVLINK_PORT: int = 14550   # UDP QGroundControl / SITL loopback

# Reference Geographic Anchor for MAVLink GPS mapping (San Francisco bay area)
HOME_LAT: float = 37.7749
HOME_LON: float = -122.4194
HOME_ALT: float = 20.0             # Meters AMSL


# ==============================================================================
# UDP & MAVLink Network Bridge
# ==============================================================================
class MavlinkBridge:
    """Manages UDP JSON telemetry broadcast, inbound command ingestion,
    and simulated binary MAVLink v2 loopback messages for external GCS/SITL."""

    def __init__(
        self,
        sim: SwarmSimulator,
        host: str = DEFAULT_HOST,
        tx_port: int = DEFAULT_TX_PORT,
        rx_port: int = DEFAULT_RX_PORT,
        mavlink_port: int = DEFAULT_MAVLINK_PORT,
        enable_mavlink: bool = True,
    ):
        self.sim: SwarmSimulator = sim
        self.host: str = host
        self.tx_port: int = tx_port
        self.rx_port: int = rx_port
        self.mavlink_port: int = mavlink_port
        self.enable_mavlink: bool = enable_mavlink

        self.last_mavlink_heartbeat: float = -1.0
        self.last_mavlink_pos: float = -1.0
        self.packet_count: int = 0
        self.commands_processed: int = 0

        # Setup TX Socket (Broadcast / Send JSON)
        self.tx_sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Setup RX Socket (Non-blocking Listener for JSON commands)
        self.rx_sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_sock.bind((self.host, self.rx_port))
        self.rx_sock.setblocking(False)

        # Setup MAVLink TX Socket
        self.mavlink_sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # MAVLink Protocol Encoder
        if HAVE_PYMAVLINK:
            self.mav = mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)
        else:
            self.mav = None

    def broadcast_telemetry(self, snapshot: Optional[Dict[str, Any]] = None) -> int:
        """Broadcast simulation state JSON snapshot to UDP port 9000.
        Returns number of bytes sent."""
        if snapshot is None:
            snapshot = self.sim.get_state_snapshot()

        payload = json.dumps(snapshot).encode("utf-8")
        bytes_sent = self.tx_sock.sendto(payload, (self.host, self.tx_port))
        self.packet_count += 1
        return bytes_sent

    def poll_commands(self) -> List[Dict[str, Any]]:
        """Non-blocking poll of UDP port 9001 for incoming command JSON packets.
        Executes recognized commands on the simulator and returns response list."""
        responses = []
        while True:
            try:
                data, addr = self.rx_sock.recvfrom(4096)
                if not data:
                    break

                try:
                    cmd_json = json.loads(data.decode("utf-8"))
                    res = self.execute_command(cmd_json)
                    res["client_addr"] = addr
                    responses.append(res)
                    self.commands_processed += 1
                except (json.JSONDecodeError, UnicodeDecodeError) as e:
                    responses.append({"status": "ERROR", "error": f"Invalid JSON: {str(e)}"})

            except BlockingIOError:
                break
            except Exception as e:
                responses.append({"status": "ERROR", "error": str(e)})
                break

        return responses

    def execute_command(self, cmd_dict: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a structured command on the underlying simulation engine."""
        cmd = cmd_dict.get("cmd", "").upper()

        if cmd in ("SET_TARGET", "DISPATCH_TARGET"):
            pos = cmd_dict.get("pos", [60.0, 40.0, 30.0])
            self.sim.patrol_active = False
            self.sim.set_target(pos)
            return {"status": "OK", "cmd": cmd, "target": pos, "patrol_active": False}

        elif cmd in ("TOGGLE_PATROL", "TOGGLE_TARGET_PATROL", "SET_PATROL"):
            active = cmd_dict.get("active", None)
            state = self.sim.toggle_target_patrol(active)
            return {"status": "OK", "cmd": cmd, "patrol_active": state}

        elif cmd == "SET_FORMATION":
            mode = cmd_dict.get("mode", "ECHELON_ESCORT")
            self.sim.set_formation(mode)
            return {"status": "OK", "cmd": cmd, "formation": self.sim.formation_mode}

        elif cmd == "KILL_LEADER":
            self.sim.kill_leader()
            new_leader_id = self.sim.leader.id if self.sim.leader else -1
            return {"status": "OK", "cmd": cmd, "new_leader_id": new_leader_id}

        elif cmd == "KILL_DRONE":
            drone_id = int(cmd_dict.get("id", 1))
            self.sim.kill_drone(drone_id)
            return {"status": "OK", "cmd": cmd, "killed_id": drone_id}

        elif cmd == "REBOOT_DRONE":
            drone_id = int(cmd_dict.get("id", 1))
            self.sim.reboot_drone(drone_id)
            return {"status": "OK", "cmd": cmd, "rebooted_id": drone_id}

        elif cmd in ("LAUNCH_REINFORCEMENT", "REINFORCE_SWARM", "SPAWN_REINFORCEMENT"):
            new_drone = self.sim.launch_reinforcement()
            return {
                "status": "OK",
                "cmd": cmd,
                "drone_id": new_drone.id,
                "total_drones": len(self.sim.drones),
            }

        elif cmd in ("REBOOT_ALL_DEAD", "RESPAWN_ALL_DEAD", "REBOOT_DEAD"):
            revived = self.sim.reboot_all_dead()
            return {
                "status": "OK",
                "cmd": cmd,
                "revived_ids": revived,
                "count": len(revived),
            }

        elif cmd in ("CLEAR_WRECKAGE", "SALVAGE_WRECKAGE", "CLEAR_CASUALTIES"):
            purged = self.sim.clear_crashed_wreckage()
            return {
                "status": "OK",
                "cmd": cmd,
                "purged_ids": purged,
                "count": len(purged),
                "remaining_drones": len(self.sim.drones),
            }

        elif cmd == "REBOOT_FLEET":
            self.sim.reboot_fleet()
            return {"status": "OK", "cmd": cmd, "drone_count": len(self.sim.drones)}

        elif cmd in ("CUT_TETHER", "JETTISON_TETHER", "SEVER_TETHER"):
            self.sim.cut_tether()
            two_ic = self.sim.get_2ic()
            return {
                "status": "OK",
                "cmd": cmd,
                "tether_severed": self.sim.tether_severed,
                "two_ic_id": two_ic.id if two_ic else None,
            }

        elif cmd in ("RTB_HOLD", "COMMAND_RTB_HOLD", "RETURN_TO_BASE_HOLD", "RTB"):
            self.sim.command_rtb_hold()
            return {
                "status": "OK",
                "cmd": cmd,
                "recovery_stage": self.sim.recovery_stage,
                "overhead_marshal_active": self.sim.overhead_marshal_active,
            }

        elif cmd in ("CANCEL_RTB", "RESUME_MISSION"):
            self.sim.cancel_rtb()
            return {
                "status": "OK",
                "cmd": cmd,
                "recovery_stage": self.sim.recovery_stage,
                "overhead_marshal_active": self.sim.overhead_marshal_active,
            }

        elif cmd in ("LAND_SWARM", "COMMAND_LAND_SWARM", "LAND_ALL"):
            queued = self.sim.command_land_swarm()
            return {
                "status": "OK",
                "cmd": cmd,
                "queued_ids": queued,
                "count": len(queued),
            }

        elif cmd in ("LAND_DRONE", "COMMAND_LAND_DRONE"):
            drone_id = int(cmd_dict.get("id", cmd_dict.get("drone_id", 0)))
            success = self.sim.command_land_drone(drone_id)
            return {"status": "OK" if success else "ERROR", "cmd": cmd, "drone_id": drone_id}

        elif cmd in ("TAKEOFF_DRONE", "RELAUNCH_DRONE"):
            drone_id = int(cmd_dict.get("id", cmd_dict.get("drone_id", 0)))
            success = self.sim.command_takeoff_drone(drone_id)
            return {"status": "OK" if success else "ERROR", "cmd": cmd, "drone_id": drone_id}

        elif cmd == "SET_BRAIN":
            name = cmd_dict.get("brain", "LEADER_FOLLOWER")
            ok = self.sim.set_brain(name)
            return {"status": "OK" if ok else "INVALID_BRAIN", "cmd": cmd, "active_brain": self.sim.active_brain_name}

        else:
            # Check active brain hook
            brain_res = self.sim.brain.handle_command(cmd_dict)
            if brain_res is not None:
                return {"status": "OK", **brain_res}
            return {"status": "UNKNOWN_COMMAND", "cmd": cmd}

    def emit_mavlink_loopback(self) -> int:
        """Emit simulated MAVLink v2 HEARTBEAT (1 Hz) and GLOBAL_POSITION_INT (10 Hz)
        messages for the active leader on port 14550."""
        if not self.enable_mavlink or not HAVE_PYMAVLINK or self.mav is None:
            return 0

        leader = self.sim.leader
        if leader is None:
            return 0

        current_time = self.sim.sim_time
        bytes_sent = 0

        # 1. HEARTBEAT at 1 Hz
        if current_time - self.last_mavlink_heartbeat >= 1.0:
            self.last_mavlink_heartbeat = current_time
            msg_hb = self.mav.heartbeat_encode(
                type=mavlink2.MAV_TYPE_QUADROTOR,
                autopilot=mavlink2.MAV_AUTOPILOT_GENERIC,
                base_mode=mavlink2.MAV_MODE_FLAG_SAFETY_ARMED | mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode=4,  # Auto / Guided
                system_status=mavlink2.MAV_STATE_ACTIVE if not leader.crashed else mavlink2.MAV_STATE_CRITICAL,
            )
            pkt_hb = msg_hb.pack(self.mav)
            bytes_sent += self.mavlink_sock.sendto(pkt_hb, (self.host, self.mavlink_port))

        # 2. GLOBAL_POSITION_INT at 10 Hz (every 0.1s)
        if current_time - self.last_mavlink_pos >= 0.1:
            self.last_mavlink_pos = current_time

            # Flat-Earth coordinate projection to GPS degrees
            # 1 deg lat ~ 111139m, 1 deg lon ~ 111139 * cos(lat)m
            lat_deg = HOME_LAT + (leader.pos[1] / 111139.0)
            lon_deg = HOME_LON + (leader.pos[0] / (111139.0 * math.cos(math.radians(HOME_LAT))))
            alt_m = HOME_ALT + leader.pos[2]
            rel_alt_m = leader.pos[2]

            # Convert heading to centidegrees [0..35999]
            yaw_deg = math.degrees(leader.yaw) % 360.0
            hdg_cdeg = int(yaw_deg * 100.0)

            msg_pos = self.mav.global_position_int_encode(
                time_boot_ms=int(current_time * 1000.0),
                lat=int(lat_deg * 1e7),
                lon=int(lon_deg * 1e7),
                alt=int(alt_m * 1000.0),
                relative_alt=int(rel_alt_m * 1000.0),
                vx=int(leader.vel[0] * 100.0),
                vy=int(leader.vel[1] * 100.0),
                vz=int(-leader.vel[2] * 100.0),  # NED convention: down is positive
                hdg=hdg_cdeg,
            )
            pkt_pos = msg_pos.pack(self.mav)
            bytes_sent += self.mavlink_sock.sendto(pkt_pos, (self.host, self.mavlink_port))

        return bytes_sent

    def step(self, snapshot: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Unified update step: polls commands, broadcasts JSON state, and sends MAVLink."""
        cmd_responses = self.poll_commands()
        tx_bytes = self.broadcast_telemetry(snapshot)
        mav_bytes = self.emit_mavlink_loopback()

        return {
            "commands_processed": len(cmd_responses),
            "command_responses": cmd_responses,
            "telemetry_bytes_sent": tx_bytes,
            "mavlink_bytes_sent": mav_bytes,
        }

    def close(self) -> None:
        """Close all network sockets cleanly."""
        try:
            self.tx_sock.close()
        except Exception:
            pass
        try:
            self.rx_sock.close()
        except Exception:
            pass
        try:
            self.mavlink_sock.close()
        except Exception:
            pass
