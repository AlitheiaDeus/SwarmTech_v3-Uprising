"""
SwarmTech3_UpRising: Web & WebSocket Telemetry Server (web_server.py)
High-performance aiohttp web server serving the WebGL Tactical GCS frontend
and streaming 60 FPS JSON simulation state frames over WebSocket (/ws).
"""

from __future__ import annotations
import asyncio
import json
import logging
import os
from pathlib import Path
import time
from typing import Dict, List, Optional, Set, Any

from aiohttp import web, WSMsgType

from backend.sim_core import SwarmSimulator, DT
from backend.mavlink_bridge import MavlinkBridge

logger = logging.getLogger("SwarmWebServer")


class SwarmWebServer:
    """aiohttp server hosting the WebGL GCS static assets and streaming
    real-time 60 FPS simulation telemetry over WebSocket (/ws)."""

    def __init__(
        self,
        sim: SwarmSimulator,
        bridge: Optional[MavlinkBridge] = None,
        host: str = "127.0.0.1",
        port: int = 8080,
        frontend_dir: Optional[Path] = None,
        fps: int = 60,
    ):
        self.sim: SwarmSimulator = sim
        self.bridge: Optional[MavlinkBridge] = bridge
        self.host: str = host
        self.port: int = port
        self.fps: int = fps
        self.dt: float = 1.0 / float(fps)

        if frontend_dir is None:
            # Default to repo root / frontend
            self.frontend_dir = Path(__file__).resolve().parent.parent / "frontend"
        else:
            self.frontend_dir = Path(frontend_dir)

        self.app: web.Application = web.Application()
        self.active_websockets: Set[web.WebSocketResponse] = set()
        self.runner: Optional[web.AppRunner] = None
        self.site: Optional[web.TCPSite] = None
        self.sim_task: Optional[asyncio.Task] = None
        self.is_running: bool = False
        self.frames_broadcast: int = 0

        self._setup_routes()

    def _setup_routes(self) -> None:
        """Configure REST, WebSocket, and static frontend file endpoints."""
        self.app.router.add_get("/ws", self.websocket_handler)
        self.app.router.add_get("/api/status", self.api_status_handler)
        self.app.router.add_post("/api/command", self.api_command_handler)

        # Static GCS frontend routes
        self.app.router.add_get("/", self.index_handler)

        if self.frontend_dir.exists():
            # Mount static assets directory
            self.app.router.add_static(
                "/static",
                path=str(self.frontend_dir),
                name="static",
                show_index=False,
            )

    async def index_handler(self, request: web.Request) -> web.Response:
        """Serve index.html from frontend directory or a fallback status page."""
        index_file = self.frontend_dir / "index.html"
        if index_file.exists():
            return web.FileResponse(str(index_file))

        fallback_html = """<!DOCTYPE html>
<html>
<head><title>SwarmTech3_UpRising Telemetry Server</title></head>
<body style="background:#061019;color:#00F0FF;font-family:monospace;padding:2rem;">
  <h2>[SwarmTech3_UpRising] Telemetry & WebGL GCS Server Active</h2>
  <p>WebSocket endpoint: <code>/ws</code></p>
  <p>Status: ONLINE | 60 FPS Vectorized Engine Active</p>
</body>
</html>"""
        return web.Response(text=fallback_html, content_type="text/html")

    async def api_status_handler(self, request: web.Request) -> web.Response:
        """Return high-level simulation status JSON."""
        return web.json_response({
            "status": "ONLINE",
            "sim_time": round(float(self.sim.sim_time), 3),
            "drone_count": len(self.sim.drones),
            "formation": self.sim.formation_mode,
            "connected_clients": len(self.active_websockets),
            "frames_broadcast": self.frames_broadcast,
        })

    async def api_command_handler(self, request: web.Request) -> web.Response:
        """Ingest commands over HTTP POST."""
        try:
            data = await request.json()
            result = self.execute_command(data)
            return web.json_response(result)
        except Exception as e:
            return web.json_response({"status": "ERROR", "error": str(e)}, status=400)

    def execute_command(self, cmd_dict: Dict[str, Any]) -> Dict[str, Any]:
        """Dispatch operator directive to simulation engine."""
        cmd = cmd_dict.get("cmd", "").upper()

        if cmd in ("SET_TARGET", "DISPATCH_TARGET"):
            pos = cmd_dict.get("pos", [60.0, 40.0, 30.0])
            self.sim.set_target(pos)
            return {"status": "OK", "cmd": "SET_TARGET", "target": pos}

        elif cmd == "SET_FORMATION":
            mode = cmd_dict.get("mode", "ECHELON_ESCORT")
            self.sim.set_formation(mode)
            return {"status": "OK", "cmd": cmd, "formation": self.sim.formation_mode}

        elif cmd == "KILL_LEADER":
            self.sim.kill_leader()
            new_id = self.sim.leader.id if self.sim.leader else -1
            return {"status": "OK", "cmd": cmd, "new_leader_id": new_id}

        elif cmd == "KILL_DRONE":
            drone_id = int(cmd_dict.get("id", 1))
            self.sim.kill_drone(drone_id)
            return {"status": "OK", "cmd": cmd, "killed_id": drone_id}

        elif cmd == "REBOOT_DRONE":
            drone_id = int(cmd_dict.get("id", 1))
            self.sim.reboot_drone(drone_id)
            return {"status": "OK", "cmd": cmd, "rebooted_id": drone_id}

        elif cmd == "REBOOT_FLEET":
            self.sim.reboot_fleet()
            return {"status": "OK", "cmd": cmd, "drone_count": len(self.sim.drones)}

        elif cmd == "CUT_TETHER":
            self.sim.cut_tether()
            return {"status": "OK", "cmd": cmd, "tether_severed": self.sim.tether_severed}

        else:
            return {"status": "UNKNOWN_COMMAND", "cmd": cmd}

    async def websocket_handler(self, request: web.Request) -> web.WebSocketResponse:
        """Handle 2-way real-time WebSocket connection:
        streams 60 FPS telemetry down, ingests commands up."""
        ws = web.WebSocketResponse(heartbeat=15.0)
        await ws.prepare(request)

        self.active_websockets.add(ws)

        # Immediately send current state on connect
        initial_snapshot = self.sim.get_state_snapshot()
        initial_snapshot["type"] = "TELEMETRY_FRAME"
        await ws.send_str(json.dumps(initial_snapshot))

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        cmd_dict = json.loads(msg.data)
                        res = self.execute_command(cmd_dict)
                        await ws.send_str(json.dumps({"type": "COMMAND_ACK", **res}))
                    except Exception as e:
                        await ws.send_str(json.dumps({"type": "ERROR", "message": str(e)}))
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            self.active_websockets.discard(ws)

        return ws

    async def _simulation_loop(self) -> None:
        """Core async loop stepping the simulation at locked 60 FPS and
        broadcasting JSON frames to all connected GCS clients."""
        interval = self.dt

        while self.is_running:
            t_start = time.perf_counter()

            # Advance simulation world
            self.sim.step(interval)

            # Step UDP / MAVLink bridge if attached
            if self.bridge is not None:
                self.bridge.step()

            # Broadcast state snapshot over WebSocket
            if self.active_websockets:
                snapshot = self.sim.get_state_snapshot()
                snapshot["type"] = "TELEMETRY_FRAME"
                payload = json.dumps(snapshot)
                stale_sockets = []

                for ws in list(self.active_websockets):
                    try:
                        if not ws.closed:
                            await ws.send_str(payload)
                        else:
                            stale_sockets.append(ws)
                    except Exception:
                        stale_sockets.append(ws)

                for stale in stale_sockets:
                    self.active_websockets.discard(stale)

            self.frames_broadcast += 1

            # Sleep remaining time to maintain target FPS
            elapsed = time.perf_counter() - t_start
            sleep_time = max(0.001, interval - elapsed)
            await asyncio.sleep(sleep_time)

    async def start(self) -> None:
        """Start web server and simulation ticker loop."""
        self.is_running = True
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()

        self.site = web.TCPSite(self.runner, self.host, self.port)
        await self.site.start()

        # Launch simulation loop as concurrent asyncio task
        self.sim_task = asyncio.create_task(self._simulation_loop())

    async def stop(self) -> None:
        """Gracefully stop server, close client sockets, and cancel sim loop."""
        self.is_running = False

        if self.sim_task is not None:
            self.sim_task.cancel()
            try:
                await self.sim_task
            except asyncio.CancelledError:
                pass
            self.sim_task = None

        # Close all active WebSockets
        for ws in list(self.active_websockets):
            try:
                await ws.close(code=1000, message=b"Server shutdown")
            except Exception:
                pass
        self.active_websockets.clear()

        # Shutdown aiohttp runner
        if self.runner is not None:
            await self.runner.cleanup()
            self.runner = None

        # Close network bridge if attached
        if self.bridge is not None:
            self.bridge.close()
