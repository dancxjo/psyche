#!/usr/bin/env bash
set -euo pipefail

. "$(dirname "$0")/_common.sh" 2>/dev/null || true

SERVICE_NAME="language"
OLLAMA_INSTALL_URL="https://ollama.com/install.sh"
DEFAULT_MODEL="${PSY_OLLAMA_MODEL:-gemma3n}"

ensure_runtime_prereqs() {
  export PSY_DEFER_APT=1
  common_apt_install curl
}

install_ollama() {
  if command -v ollama >/dev/null 2>&1; then
    echo "[language] Ollama already installed"
    return 0
  fi

  ensure_runtime_prereqs
  echo "[language] Installing Ollama from ${OLLAMA_INSTALL_URL}"
  curl -fsSL "${OLLAMA_INSTALL_URL}" | sh
}

enable_ollama_service() {
  if ! command -v systemctl >/dev/null 2>&1; then
    echo "[language] systemctl not available; skipping Ollama service enablement" >&2
    return 0
  fi

  sudo systemctl enable --now ollama >/dev/null 2>&1 || {
    echo "[language] WARNING: Failed to enable/start ollama.service" >&2
    return 1
  }

  return 0
}

wait_for_ollama() {
  local max_attempts=30
  local delay=2
  local attempt=1
  local host="${OLLAMA_HOST:-http://127.0.0.1:${OLLAMA_PORT:-11434}}"

  while [ "$attempt" -le "$max_attempts" ]; do
    if curl -fsS "${host}/api/version" >/dev/null 2>&1; then
      return 0
    fi
    sleep "$delay"
    attempt=$((attempt + 1))
  done

  echo "[language] WARNING: Ollama service did not become ready" >&2
  return 1
}

fetch_default_model() {
  local model="${DEFAULT_MODEL}"
  if [ -z "$model" ]; then
    echo "[language] No default model configured; skipping prefetch"
    return 0
  fi

  echo "[language] Prefetching Ollama model: ${model}"
  if ! echo "Hello" | ollama run "$model" >/tmp/psyched-ollama-${model}.log 2>&1; then
    echo "[language] WARNING: ollama run ${model} failed (see /tmp/psyched-ollama-${model}.log)" >&2
    return 1
  fi
  return 0
}

install_launcher() {
  common_install_launcher "$SERVICE_NAME" LAUNCH <<'LAUNCH'
#!/usr/bin/env bash
set -euo pipefail

if ! command -v ollama >/dev/null 2>&1; then
  echo "[language] Ollama CLI missing" >&2
  exit 1
fi

if command -v systemctl >/dev/null 2>&1; then
  systemctl start ollama >/dev/null 2>&1 || true
fi

if command -v journalctl >/dev/null 2>&1; then
  exec journalctl -f -u ollama.service
fi

exec tail -f /var/log/ollama.log 2>/dev/null || exec sleep infinity
LAUNCH
}

provision() {
  install_ollama || true
  enable_ollama_service || true
  wait_for_ollama || true
  fetch_default_model || true
  install_launcher
}

case "${1:-provision}" in
  provision)
    provision
    ;;
  *)
    echo "unknown subcmd" >&2
    exit 1
    ;;
esac
