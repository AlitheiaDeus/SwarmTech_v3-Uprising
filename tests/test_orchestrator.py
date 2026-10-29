"""
SwarmTech3_UpRising: Multi-Process Orchestrator Unit Tests (test_orchestrator.py)
Tests run_with_sim.py CLI argument parsing, smoke test runner, and child cleanup logic.
"""

from __future__ import annotations
import asyncio
import sys
import unittest
from pathlib import Path
from unittest.mock import patch, MagicMock

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import run_with_sim


class TestOrchestrator(unittest.TestCase):
    """Test suite validating run_with_sim.py orchestrator capabilities."""

    def test_argument_parser_defaults(self):
        """Verify default argument configuration for full-stack launch."""
        with patch.object(sys, "argv", ["run_with_sim.py"]):
            args = run_with_sim.parse_arguments()
            self.assertEqual(args.count, 8)
            self.assertEqual(args.port, 8080)
            self.assertEqual(args.tx_port, 9000)
            self.assertEqual(args.rx_port, 9001)
            self.assertEqual(args.mavlink_port, 14550)
            self.assertFalse(args.smoke_test)
            self.assertFalse(args.sim_only)
            self.assertFalse(args.gcs_only)
            self.assertFalse(args.multi_process)

    def test_argument_parser_modular_modes(self):
        """Verify CLI flags for modular execution modes."""
        with patch.object(sys, "argv", ["run_with_sim.py", "--sim-only", "--count", "12"]):
            args = run_with_sim.parse_arguments()
            self.assertTrue(args.sim_only)
            self.assertEqual(args.count, 12)

        with patch.object(sys, "argv", ["run_with_sim.py", "--gcs-only", "--port", "9090"]):
            args = run_with_sim.parse_arguments()
            self.assertTrue(args.gcs_only)
            self.assertEqual(args.port, 9090)

        with patch.object(sys, "argv", ["run_with_sim.py", "--multi-process"]):
            args = run_with_sim.parse_arguments()
            self.assertTrue(args.multi_process)

    def test_smoke_test_execution(self):
        """Verify run_smoke_test executes and exits with code 0."""
        with patch.object(sys, "argv", ["run_with_sim.py", "--smoke-test", "--count", "4"]):
            args = run_with_sim.parse_arguments()
            code = asyncio.run(run_with_sim.run_smoke_test(args))
            self.assertEqual(code, 0)

    def test_cleanup_children_logic(self):
        """Verify child subprocesses are terminated cleanly without exceptions."""
        mock_proc = MagicMock()
        mock_proc.poll.return_value = None
        run_with_sim._CHILD_PROCESSES.append(mock_proc)

        run_with_sim._cleanup_children()
        mock_proc.terminate.assert_called_once()
        self.assertEqual(len(run_with_sim._CHILD_PROCESSES), 0)


if __name__ == "__main__":
    unittest.main()
