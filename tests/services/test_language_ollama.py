"""Tests ensuring the language provisioning script installs and prepares Ollama."""

from pathlib import Path


LANGUAGE_SERVICE = Path("provision/services/language.sh")


def test_language_installs_ollama_from_official_script():
    script = LANGUAGE_SERVICE.read_text()
    assert 'OLLAMA_INSTALL_URL="https://ollama.com/install.sh"' in script
    assert 'curl -fsSL "${OLLAMA_INSTALL_URL}" | sh' in script


def test_language_enables_ollama_service():
    script = LANGUAGE_SERVICE.read_text()
    assert "systemctl enable --now ollama" in script


def test_language_prefetches_gemma3n():
    script = LANGUAGE_SERVICE.read_text()
    assert 'DEFAULT_MODEL="${PSY_OLLAMA_MODEL:-gemma3n}"' in script
    assert "ollama run" in script
