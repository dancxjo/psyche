"""Behavior-driven tests verifying Docker provisioning for the AI stack."""

from pathlib import Path


COMMON_HELPERS = Path("provision/services/_common.sh")
AI_STACK_SERVICE = Path("provision/services/ai_stack.sh")


def test_ai_stack_uses_common_docker_runtime_helper():
    """Given the AI stack service, it should delegate Docker setup to the shared helper."""
    script = AI_STACK_SERVICE.read_text()
    assert "common_ensure_docker_runtime" in script


def test_common_helper_configures_official_docker_repository():
    """When preparing Docker, the helper must reference the official Docker APT repository."""
    helpers = COMMON_HELPERS.read_text()
    assert "download.docker.com" in helpers
    assert "/etc/apt/sources.list.d/docker.list" in helpers


def test_common_helper_installs_docker_ce_and_compose():
    """Then the helper should install the Docker CE engine and compose plugin packages."""
    helpers = COMMON_HELPERS.read_text()
    assert "docker-ce" in helpers
    assert "docker-compose-plugin" in helpers
