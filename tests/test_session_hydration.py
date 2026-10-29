"""
SwarmTech3_UpRising: Persistent Telemetry Session & Reconnect Hydration Tests (test_session_hydration.py)
Validates TKT-32:
  1. Circular in-memory event history buffer (max 100 entries, FIFO eviction).
  2. Full GCS state hydration bundle extraction.
  3. WebSocket initial hydration transmission on connection to eliminate browser refresh amnesia.
"""

from __future__ import annotations
import asyncio
import json
import sys
import unittest
from pathlib import Path
import aiohttp

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from GCS.backend.session_manager import SessionManager
from GCS.backend.web_server import SwarmWebServer
from SIM.backend.sitl_engine import SITLWorldEngine


class TestSessionHydration(unittest.IsolatedAsyncioTestCase):
    """Test suite verifying persistent session management and WebSocket state hydration."""

    def setUp(self):
        self.mgr = SessionManager(max_events=100)

    def test_session_manager_initial_event(self):
        """Verify session manager initializes with a default SYSTEM event."""
        history = self.mgr.get_event_history()
        self.assertEqual(len(history), 1)
        self.assertEqual(history[0]["type"], "SYSTEM")
        self.assertIn("Initialized", history[0]["message"])
        self.assertIsNotNone(history[0]["wall_time"])

    def test_circular_event_buffer_capping(self):
        """Verify circular event history caps strictly at max_events (100) with FIFO eviction."""
        self.mgr.clear()
        self.assertEqual(len(self.mgr.get_event_history()), 0)

        # Add 120 events
        for i in range(120):
            self.mgr.add_event("CMD", f"Event #{i}", sim_time=float(i))

        history = self.mgr.get_event_history()
        self.assertEqual(len(history), 100)
        # Oldest event should be #20 (first 20 evicted)
        self.assertEqual(history[0]["message"], "Event #20")
        self.assertEqual(history[-1]["message"], "Event #119")

    def test_hydration_bundle_composition(self):
        """Verify hydration bundle correctly serializes live sim state and historical events."""
        engine = SITLWorldEngine(drone_count=4, seed=42)
        engine.set_comms_mode("GOSSIP_CONSENSUS")
        engine.set_formation("ORBIT_VORTEX")
        engine.set_rf_jamming(0.25)
        engine.kill_drone(2)

        self.mgr.add_event("CASUALTY", "Drone #2 destroyed", sim_time=1.5)
        bundle = self.mgr.get_hydration_bundle(engine)

        self.assertEqual(bundle["type"], "SESSION_HYDRATION")
        self.assertEqual(bundle["comms_mode"], "GOSSIP_CONSENSUS")
        self.assertEqual(bundle["formation"], "ORBIT_VORTEX")
        self.assertEqual(bundle["rf_jamming"], 0.25)
        self.assertIn(2, bundle["crashed_ids"])
        self.assertGreaterEqual(len(bundle["event_history"]), 2)

    async def test_websocket_hydration_on_connect(self):
        """Verify newly connected WebSocket receives SESSION_HYDRATION followed by TELEMETRY_FRAME."""
        engine = SITLWorldEngine(drone_count=4, seed=42)
        server = SwarmWebServer(sim=engine, host="127.0.0.1", port=18089)

        # Pre-seed server session events
        server.session_manager.add_event("CMD", "Pre-connect route deployed", sim_time=2.0)
        server.session_manager.add_event("FAILSAFE", "Pre-connect tether warning", sim_time=3.0)

        await server.start()

        try:
            async with aiohttp.ClientSession() as session:
                async with session.ws_connect("http://127.0.0.1:18089/ws") as ws:
                    # First message MUST be SESSION_HYDRATION
                    msg1 = await ws.receive_json(timeout=2.0)
                    self.assertEqual(msg1["type"], "SESSION_HYDRATION")
                    self.assertIn("event_history", msg1)
                    history_messages = [e["message"] for e in msg1["event_history"]]
                    self.assertIn("Pre-connect route deployed", history_messages)
                    self.assertIn("Pre-connect tether warning", history_messages)

                    # Second message MUST be TELEMETRY_FRAME
                    msg2 = await ws.receive_json(timeout=2.0)
                    self.assertEqual(msg2["type"], "TELEMETRY_FRAME")
                    self.assertIn("drones", msg2)
                    self.assertEqual(len(msg2["drones"]), 4)
        finally:
            await server.stop()


if __name__ == "__main__":
    unittest.main()
