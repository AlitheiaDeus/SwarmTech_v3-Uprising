"""
SwarmTech3_UpRising: Web & WebSocket Telemetry Server (web_server.py)
High-performance aiohttp web server serving the WebGL Tactical GCS frontend
and streaming 60 FPS JSON simulation state frames over WebSocket (/ws).
"""

from __future__ import annotations
import asyncio
import ctypes
import json
import logging
import os
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional, Set, Any

# Ensure repository root is on sys.path even when executed directly via file path
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# Self-healing Conda Windows DLL path bootstrap:
if sys.platform == "win32":
    _prefix = Path(sys.prefix)
    _dll_candidates = [
        _prefix / "Library" / "bin",
        _prefix / "Scripts",
        _prefix.parent.parent / "Library" / "bin",
    ]
    for _d in _dll_candidates:
        if _d.exists():
            _p_str = str(_d)
            if _p_str not in os.environ.get("PATH", ""):
                os.environ["PATH"] = _p_str + os.pathsep + os.environ.get("PATH", "")
            try:
                os.add_dll_directory(_p_str)
            except (AttributeError, OSError):
                pass

from aiohttp import web, WSMsgType

try:
    from SIM.backend.sitl_engine import SwarmSimulator, DT
    from SIM.backend.mavlink_bridge import MavlinkBridge
except ImportError:
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
            candidate_gcs = Path(__file__).resolve().parent.parent / "frontend"
            if candidate_gcs.exists() and (candidate_gcs / "index.html").exists():
                self.frontend_dir = candidate_gcs
            else:
                self.frontend_dir = Path(__file__).resolve().parent.parent.parent / "frontend"
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

        # Ensure frontend directories exist
        self.frontend_dir.mkdir(parents=True, exist_ok=True)
        css_dir = self.frontend_dir / "css"
        css_dir.mkdir(parents=True, exist_ok=True)
        js_dir = self.frontend_dir / "js"
        js_dir.mkdir(parents=True, exist_ok=True)

        self.app.router.add_static("/static", path=str(self.frontend_dir), name="static", show_index=False)
        self.app.router.add_static("/css", path=str(css_dir), name="css", show_index=False)
        self.app.router.add_static("/js", path=str(js_dir), name="js", show_index=False)

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
            self.sim.patrol_active = False
            res = self.sim.set_target(pos)
            if isinstance(res, dict) and res.get("status") in ("OUT_OF_RANGE", "DROPPED"):
                return {
                    "status": res.get("status"),
                    "cmd": cmd,
                    "error": res.get("error"),
                    "reason": res.get("reason"),
                }
            return {"status": "OK", "cmd": "SET_TARGET", "target": pos, "patrol_active": False}

        elif cmd in ("TOGGLE_PATROL", "TOGGLE_TARGET_PATROL", "SET_PATROL"):
            active = cmd_dict.get("active", None)
            state = self.sim.toggle_target_patrol(active)
            return {"status": "OK", "cmd": cmd, "patrol_active": state}

        elif cmd in ("TOGGLE_RNG_RECON", "TOGGLE_RNG", "RNG_RECON"):
            active = cmd_dict.get("active", None)
            state = self.sim.toggle_rng_recon(active)
            return {"status": "OK", "cmd": cmd, "rng_recon_active": state}

        elif cmd in ("START_RNG_RECON", "START_RNG"):
            return self.sim.start_rng_recon()

        elif cmd in ("STOP_RNG_RECON", "STOP_RNG"):
            return self.sim.stop_rng_recon()

        elif cmd in ("START_ROUTE_PRESET", "SET_ROUTE_PRESET"):
            preset = cmd_dict.get("preset", cmd_dict.get("name", "BOX_PATROL"))
            res = self.sim.start_route_preset(preset)
            return res

        elif cmd in ("UPLOAD_MISSION_ROUTE", "DEPLOY_ROUTE", "SET_MISSION_ROUTE"):
            waypoints = cmd_dict.get("waypoints", [])
            loop = bool(cmd_dict.get("loop", True))
            name = str(cmd_dict.get("name", "CUSTOM"))
            res = self.sim.deploy_mission_route(waypoints, loop=loop, name=name)
            return res

        elif cmd in ("STOP_MISSION_ROUTE", "STOP_ROUTE", "ABORT_ROUTE"):
            res = self.sim.stop_mission_route()
            return res

        elif cmd in ("SET_COMMS_MODE", "SET_MODE"):
            mode = cmd_dict.get("mode", "TETHERED_RELAY")
            ok = self.sim.set_comms_mode(mode)
            return {
                "status": "OK" if ok else "ERROR",
                "cmd": cmd,
                "comms_mode": self.sim.comms_mode,
                "active_brain": self.sim.active_brain_name,
            }

        elif cmd in ("SET_RF_JAMMING", "SET_JAMMING"):
            rate = float(cmd_dict.get("rate", cmd_dict.get("jamming", 0.0)))
            jamming = self.sim.set_rf_jamming(rate)
            return {
                "status": "OK",
                "cmd": cmd,
                "rf_jamming": jamming,
            }

        elif cmd == "SET_FORMATION":
            mode = cmd_dict.get("mode", "ECHELON_ESCORT")
            self.sim.set_formation(mode)
            return {"status": "OK", "cmd": cmd, "formation": self.sim.formation_mode}

        elif cmd == "KILL_LEADER":
            self.sim.kill_leader()
            new_id = self.sim.leader.id if self.sim.leader else -1
            return {"status": "OK", "cmd": cmd, "new_leader_id": new_id}

        elif cmd in ("CUT_TETHER", "JETTISON_TETHER", "SEVER_TETHER"):
            self.sim.cut_tether()
            return {
                "status": "OK",
                "cmd": cmd,
                "tether_severed": self.sim.tether_severed,
            }

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
            return {
                "status": "OK" if ok else "INVALID_BRAIN",
                "cmd": cmd,
                "active_brain": self.sim.active_brain_name,
                "comms_mode": self.sim.comms_mode,
            }

        else:
            brain_res = self.sim.brain.handle_command(cmd_dict)
            if brain_res is not None:
                return {"status": "OK", **brain_res}
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
        next_tick = time.perf_counter()

        while self.is_running:
            # Advance GCS C2 layer and physics simulation world
            self.sim.step_c2(interval)
            self.sim.step(interval, advance_c2=False)

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

            # Precision frame pacing with zero clock drift
            next_tick += interval
            now = time.perf_counter()
            sleep_time = next_tick - now
            if sleep_time > 0.001:
                await asyncio.sleep(sleep_time)
            else:
                if sleep_time < -interval:
                    next_tick = now  # Reset deadline if running significantly behind
                await asyncio.sleep(0)  # Yield control to event loop

    async def start(self) -> None:
        """Start web server and simulation ticker loop."""
        # Boost Windows multimedia timer resolution to 1ms
        if sys.platform == "win32":
            try:
                ctypes.windll.winmm.timeBeginPeriod(1)
            except Exception as e:
                logger.warning(f"Could not enable Windows 1ms timer resolution: {e}")

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

        # Reset Windows multimedia timer resolution
        if sys.platform == "win32":
            try:
                ctypes.windll.winmm.timeEndPeriod(1)
            except Exception:
                pass


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="SwarmTech3 Tactical GCS Web Server")
    parser.add_argument("--port", type=int, default=8080, help="Web server port")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Web server host")
    parser.add_argument("--count", type=int, default=8, help="Number of drones")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    sim = SwarmSimulator(drone_count=args.count)
    server = SwarmWebServer(sim=sim, host=args.host, port=args.port)

    async def main():
        await server.start()
        print(f"SwarmTech3 WebGL GCS running at http://{args.host}:{args.port}", flush=True)
        try:
            while True:
                await asyncio.sleep(1)
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            await server.stop()

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
