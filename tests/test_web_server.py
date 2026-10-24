"""
Automated Unit Tests for SwarmTech3_UpRising Web & WebSocket Server (web_server.py)
Tests static file serving, status API, WebSocket telemetry streaming (60 FPS),
and bidirectional command ingestion.
"""

import asyncio
import json
import unittest
from aiohttp import ClientSession, WSMsgType

from backend.sim_core import SwarmSimulator
from backend.web_server import SwarmWebServer


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


if __name__ == "__main__":
    unittest.main()
