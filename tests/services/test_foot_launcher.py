"""Behavioral tests for the foot service launcher.

These tests act as a safety net to ensure the foot service keeps invoking the
expected ros2 launch command. We intentionally phrase the assertions using a
Given/When/Then structure to keep the desired behavior explicit.
"""

from pathlib import Path


def _read_launcher_snippet() -> str:
    """Return the foot launcher installation snippet as plain text."""

    # Given the repository root (two directories up from this test module)
    repo_root = Path(__file__).resolve().parents[2]

    # When we load the provision script for the foot service
    script_path = repo_root / "provision" / "services" / "foot.sh"
    script_text = script_path.read_text(encoding="utf-8")

    # Then we focus on the launcher section that systemd executes
    start_token = "common_install_launcher foot"
    start_index = script_text.index(start_token)
    return script_text[start_index:]


def test_launcher_executes_create_bringup() -> None:
    """The foot launcher should invoke the expected ros2 launch entrypoint."""

    snippet = _read_launcher_snippet()

    # Given the launcher snippet captured from the provision script
    # When we look for the ros2 launch invocation
    # Then it should launch the create_bringup create_1 launch file by default
    assert "exec ros2 launch" in snippet
    assert 'CREATE_LAUNCH_PACKAGE="${CREATE_LAUNCH_PACKAGE:-create_bringup}"' in snippet
    assert 'CREATE_LAUNCH_FILE="${CREATE_LAUNCH_FILE:-create_1.launch}"' in snippet
