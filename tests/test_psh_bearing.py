#!/usr/bin/env python3
"""Behavioral tests for the ``psh bearing`` command.

These tests exercise the CLI end-to-end by stubbing out the ``ros2`` executable
so we can observe the command that would have been published to ``/cmd_vel``.
The tests intentionally avoid depending on ROS 2 or the real workspace so that
they can run quickly in CI and local development environments.
"""

from __future__ import annotations

import math
import os
import re
import subprocess
import tempfile
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parent.parent


class BearingCommandTest(unittest.TestCase):
    """End-to-end coverage of the ``psh bearing`` helper."""

    def _run_bearing(self, bearing: float, *extra_args: str):
        """Invoke ``psh bearing`` with a stub ``ros2`` executable."""

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            ros2_log = tmp_path / "ros2.log"
            ros2_stub = tmp_path / "ros2"
            ros2_stub.write_text(
                f"""#!/usr/bin/env bash
set -e
# Log each argument on its own line for easy assertions.
{{ printf '%s\\n' "$@"; }} >> "{ros2_log}"
"""
            )
            ros2_stub.chmod(0o755)

            env = os.environ.copy()
            env["PATH"] = f"{tmp_path}:{env.get('PATH', '')}"

            cmd = [
                "bash",
                str(REPO_ROOT / "cli" / "psy"),
                "bearing",
            ]
            if extra_args:
                cmd.extend(extra_args)
            cmd.append(str(bearing))

            completed = subprocess.run(
                cmd,
                cwd=REPO_ROOT,
                env=env,
                check=True,
                capture_output=True,
                text=True,
            )

            log_contents = ros2_log.read_text().strip().splitlines()

        return completed, log_contents

    def test_bearing_generates_expected_twist(self):
        """A moderate bearing should result in a proportional angular velocity."""

        result, lines = self._run_bearing(10.0)

        self.assertEqual(lines[:3], ["topic", "pub", "--once"])
        self.assertEqual(lines[3], "/cmd_vel")
        self.assertEqual(lines[4], "geometry_msgs/msg/Twist")
        self.assertEqual(len(lines), 6)

        twist = lines[5]
        self.assertIn("linear: {x: 0.0, y: 0.0, z: 0.0}", twist)
        match = re.search(r"angular: \{x: 0\.0, y: 0\.0, z: ([^}]+)\}", twist)
        self.assertIsNotNone(match, f"Failed to parse angular velocity from: {twist}")
        angular_z = float(match.group(1))

        expected = 2.0 * math.radians(10.0)
        self.assertTrue(
            math.isclose(angular_z, expected, rel_tol=1e-6, abs_tol=1e-6),
            f"Expected angular velocity {expected}, got {angular_z}",
        )

        self.assertIn("angular.z=0.349066", result.stdout)
        self.assertIn("bearing=10.00°", result.stdout)

    def test_bearing_honours_deadzone(self):
        """Small bearings inside the tolerance should command a stop."""

        result, lines = self._run_bearing(1.0)
        twist = lines[5]
        match = re.search(r"angular: \{x: 0\.0, y: 0\.0, z: ([^}]+)\}", twist)
        self.assertIsNotNone(match, f"Failed to parse angular velocity from: {twist}")
        angular_z = float(match.group(1))
        self.assertTrue(math.isclose(angular_z, 0.0, abs_tol=1e-9))

        self.assertIn("within tolerance", result.stdout.lower())
        self.assertIn("angular.z=0.000000", result.stdout)


if __name__ == "__main__":  # pragma: no cover - convenience for direct execution
    unittest.main()
