"""
Automated Unit Tests for SwarmTech3_UpRising Web & WebSocket Server (web_server.py)
Tests static file serving, status API, WebSocket telemetry streaming (60 FPS),
and bidirectional command ingestion.
"""

import asyncio
import json
import unittest
from aiohttp import ClientSession, WSMsgType

from SIM.backend.sitl_engine import SwarmSimulator
from GCS.backend.web_server import SwarmWebServer


class TestWebServer(unittest.IsolatedAsyncioTestCase):
    """Async test suite verifying aiohttp server and WebSocket telemetry bridge."""

    async def asyncSetUp(self):
        self.sim = SwarmSimulator(drone_count=5, enable_packet_loss=False, seed=42)
        self.port = 18080
        self.server = SwarmWebServer(
            sim=self.sim,
            bridge=None,
            host="127.0.0.1",
            port=self.port,
            fps=60,
        )
        await self.server.start()

    async def asyncTearDown(self):
        await self.server.stop()

    async def test_api_status_endpoint(self):
        """Verify GET /api/status returns operational simulation state."""
        url = f"http://127.0.0.1:{self.port}/api/status"
        async with ClientSession() as session:
            async with session.get(url) as resp:
                self.assertEqual(resp.status, 200)
                data = await resp.json()
                self.assertEqual(data["status"], "ONLINE")
                self.assertEqual(data["drone_count"], 5)
                self.assertIn("sim_time", data)
                self.assertIn("formation", data)

    async def test_api_command_post(self):
        """Verify POST /api/command modifies simulation state."""
        url = f"http://127.0.0.1:{self.port}/api/command"
        async with ClientSession() as session:
            payload = {"cmd": "SET_TARGET", "pos": [75.0, 45.0, 35.0]}
            async with session.post(url, json=payload) as resp:
                self.assertEqual(resp.status, 200)
                res = await resp.json()
                self.assertEqual(res["status"], "OK")
                self.assertEqual(res["target"], [75.0, 45.0, 35.0])
                self.assertEqual(list(self.sim.target_pos), [75.0, 45.0, 35.0])

    async def test_static_frontend_serving(self):
        """Verify serving static index.html and frontend assets."""
        async with ClientSession() as session:
            # Check root index.html
            async with session.get(f"http://127.0.0.1:{self.port}/") as resp:
                self.assertEqual(resp.status, 200)
                html = await resp.text()
                self.assertIn("SwarmTech3_UpRising", html)
                self.assertIn("viewport-container", html)
                self.assertIn("left-panel", html)
                self.assertIn("right-panel", html)
                self.assertIn("event-feed-panel", html)

            # Check js/app.js
            async with session.get(f"http://127.0.0.1:{self.port}/js/app.js") as resp:
                self.assertEqual(resp.status, 200)
                js = await resp.text()
                self.assertIn("THREE.WebGLRenderer", js)

            # Check js/hud.js
            async with session.get(f"http://127.0.0.1:{self.port}/js/hud.js") as resp:
                self.assertEqual(resp.status, 200)
                hud_js = await resp.text()
                self.assertIn("tacticalHud", hud_js)

    async def test_websocket_telemetry_streaming(self):
        """Verify WebSocket client receives continuous 60 FPS state frames."""
        ws_url = f"http://127.0.0.1:{self.port}/ws"
        async with ClientSession() as session:
            async with session.ws_connect(ws_url) as ws:
                # Receive initial frame
                msg1 = await ws.receive()
                self.assertEqual(msg1.type, WSMsgType.TEXT)
                frame1 = json.loads(msg1.data)
                self.assertIn("sim_time", frame1)
                self.assertIn("drones", frame1)
                self.assertEqual(len(frame1["drones"]), 5)

                # Receive subsequent streamed frame from sim loop
                msg2 = await ws.receive()
                self.assertEqual(msg2.type, WSMsgType.TEXT)
                frame2 = json.loads(msg2.data)
                self.assertIn("sim_time", frame2)
                self.assertGreaterEqual(frame2["sim_time"], frame1["sim_time"])

    async def test_websocket_command_ingestion(self):
        """Verify sending command through WebSocket updates simulator and returns ACK."""
        ws_url = f"http://127.0.0.1:{self.port}/ws"

        async def wait_for_ack(ws, expected_cmd: str):
            for _ in range(20):
                msg = await asyncio.wait_for(ws.receive(), timeout=1.0)
                if msg.type == WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    if data.get("type") == "COMMAND_ACK" and data.get("cmd") == expected_cmd:
                        return data
            raise TimeoutError(f"COMMAND_ACK for {expected_cmd} not received")

        async with ClientSession() as session:
            async with session.ws_connect(ws_url) as ws:
                # Flush initial frame
                await ws.receive()

                # Send SET_FORMATION command
                cmd = {"cmd": "SET_FORMATION", "mode": "ORBIT_VORTEX"}
                await ws.send_str(json.dumps(cmd))

                # Expect ACK message
                ack_data = await wait_for_ack(ws, "SET_FORMATION")
                self.assertEqual(ack_data.get("status"), "OK")
                self.assertEqual(self.sim.formation_mode, "ORBIT_VORTEX")

                # Send KILL_LEADER command
                cmd_kill = {"cmd": "KILL_LEADER"}
                await ws.send_str(json.dumps(cmd_kill))

                ack_kill_data = await wait_for_ack(ws, "KILL_LEADER")
                self.assertEqual(ack_kill_data.get("status"), "OK")
                self.assertTrue(self.sim.drones[0].crashed)

                # Send CUT_TETHER command
                cmd_cut = {"cmd": "CUT_TETHER"}
                await ws.send_str(json.dumps(cmd_cut))
                ack_cut = await wait_for_ack(ws, "CUT_TETHER")
                self.assertEqual(ack_cut.get("status"), "OK")
                self.assertTrue(self.sim.tether_severed)

                # Send REBOOT_ALL_DEAD command
                cmd_reboot = {"cmd": "REBOOT_ALL_DEAD"}
                await ws.send_str(json.dumps(cmd_reboot))

                ack_reboot_data = await wait_for_ack(ws, "REBOOT_ALL_DEAD")
                self.assertEqual(ack_reboot_data.get("status"), "OK")
                self.assertIn(0, ack_reboot_data.get("revived_ids", []))
                self.assertFalse(self.sim.drones[0].crashed)

                # Send LAUNCH_REINFORCEMENT command
                initial_count = len(self.sim.drones)
                cmd_reinforce = {"cmd": "LAUNCH_REINFORCEMENT"}
                await ws.send_str(json.dumps(cmd_reinforce))

                ack_reinforce = await wait_for_ack(ws, "LAUNCH_REINFORCEMENT")
                self.assertEqual(ack_reinforce.get("status"), "OK")
                self.assertEqual(ack_reinforce.get("drone_id"), initial_count)
                self.assertEqual(len(self.sim.drones), initial_count + 1)
                self.assertEqual(self.sim.drones[-1].flight_state, "TAKEOFF")

                # Send KILL_DRONE command to kill drone #1
                cmd_kill_drone = {"cmd": "KILL_DRONE", "id": 1}
                await ws.send_str(json.dumps(cmd_kill_drone))
                ack_kill = await wait_for_ack(ws, "KILL_DRONE")
                self.assertEqual(ack_kill.get("status"), "OK")
                self.assertTrue(any(d.id == 1 and d.crashed for d in self.sim.drones))

                # Send CLEAR_WRECKAGE command
                count_before_clear = len(self.sim.drones)
                cmd_clear = {"cmd": "CLEAR_WRECKAGE"}
                await ws.send_str(json.dumps(cmd_clear))

                ack_clear = await wait_for_ack(ws, "CLEAR_WRECKAGE")
                self.assertEqual(ack_clear.get("status"), "OK")
                self.assertIn(1, ack_clear.get("purged_ids", []))
                self.assertFalse(any(d.id == 1 for d in self.sim.drones))
                self.assertEqual(len(self.sim.drones), count_before_clear - 1)

                # Send TOGGLE_PATROL command
                cmd_patrol = {"cmd": "TOGGLE_PATROL"}
                await ws.send_str(json.dumps(cmd_patrol))

                ack_patrol = await wait_for_ack(ws, "TOGGLE_PATROL")
                self.assertEqual(ack_patrol.get("status"), "OK")
                self.assertTrue(ack_patrol.get("patrol_active"))
                self.assertTrue(self.sim.patrol_active)

                # Send RTB_HOLD command (TKT-19)
                cmd_rtb = {"cmd": "RTB_HOLD"}
                await ws.send_str(json.dumps(cmd_rtb))
                ack_rtb = await wait_for_ack(ws, "RTB_HOLD")
                self.assertEqual(ack_rtb.get("status"), "OK")
                self.assertTrue(self.sim.overhead_marshal_active)

                # Send LAND_SWARM command (TKT-19)
                cmd_land = {"cmd": "LAND_SWARM"}
                await ws.send_str(json.dumps(cmd_land))
                ack_land = await wait_for_ack(ws, "LAND_SWARM")
                self.assertEqual(ack_land.get("status"), "OK")
                self.assertEqual(self.sim.recovery_stage, "LANDING")

    async def test_sim_loop_fps_pacing(self):
        """Verify simulation ticker loop maintains locked ~60 FPS pacing without 30 FPS quantum drop."""
        import time
        start_time = time.perf_counter()
        initial_frames = self.server.frames_broadcast
        # Wait 0.4 seconds
        await asyncio.sleep(0.4)
        elapsed = time.perf_counter() - start_time
        frames = self.server.frames_broadcast - initial_frames

        fps = frames / elapsed
        # With default 15.6ms sleep jitter, FPS dropped to ~32 FPS.
        # With 1ms WinMM timer, FPS must be >= 50 FPS.
        self.assertGreaterEqual(fps, 45.0, f"Expected >= 45 FPS, got {fps:.1f} FPS")


if __name__ == "__main__":
    unittest.main()
