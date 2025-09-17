#!/usr/bin/env bash
set -euo pipefail

. "$(dirname "$0")/_common.sh" 2>/dev/null || true

STACK_NAME="ai-stack"
ROOT_DIR="${PSY_ROOT:-/opt/psyched}"
STACK_DIR="${ROOT_DIR%/}/docker/${STACK_NAME}"
COMPOSE_FILE="${STACK_DIR}/docker-compose.yml"
ENV_FILE="/etc/default/psyched-${STACK_NAME}"

ensure_docker_runtime() {
  export PSY_DEFER_APT=1
  # Queue Docker Engine and the compose plugin; rely on host-wide flush later.
  common_apt_install docker.io docker-compose-plugin
  # Helpful but optional tools when working with Docker.
  common_apt_install ?docker-buildx-plugin ?containerd
}

write_env_defaults() {
  if [ ! -f "$ENV_FILE" ]; then
    sudo tee "$ENV_FILE" >/dev/null <<'ENV'
# psy AI stack defaults (customise as needed)
OLLAMA_MODEL=phi4
OLLAMA_PORT=11434
NEO4J_AUTH=neo4j/test
NEO4J_BOLT_PORT=7687
NEO4J_HTTP_PORT=7474
QDRANT_HTTP_PORT=6333
QDRANT_GRPC_PORT=6334
COQUI_MODEL=tts_models/en/vctk/vits
COQUI_HTTP_PORT=5002
COQUI_TTS_USE_CUDA=0
ENV
    sudo chmod 0644 "$ENV_FILE"
  fi
}

write_compose_file() {
  sudo mkdir -p "$STACK_DIR"
  sudo tee "$COMPOSE_FILE" >/dev/null <<'COMPOSE'
services:
  ollama:
    image: ollama/ollama:latest
    container_name: psy-ollama
    restart: unless-stopped
    ports:
      - "${OLLAMA_PORT:-11434}:11434"
    environment:
      - OLLAMA_HOST=0.0.0.0
      - OLLAMA_DEFAULT_MODEL=${OLLAMA_MODEL:-phi4}
    volumes:
      - ollama_data:/root/.ollama

  neo4j:
    image: neo4j:5.21.0-community
    container_name: psy-neo4j
    restart: unless-stopped
    ports:
      - "${NEO4J_HTTP_PORT:-7474}:7474"
      - "${NEO4J_BOLT_PORT:-7687}:7687"
    environment:
      - NEO4J_AUTH=${NEO4J_AUTH:-neo4j/test}
    volumes:
      - neo4j_data:/data
      - neo4j_logs:/logs
      - neo4j_plugins:/plugins

  qdrant:
    image: qdrant/qdrant:latest
    container_name: psy-qdrant
    restart: unless-stopped
    ports:
      - "${QDRANT_HTTP_PORT:-6333}:6333"
      - "${QDRANT_GRPC_PORT:-6334}:6334"
    volumes:
      - qdrant_data:/qdrant/storage

  coqui-tts:
    image: ghcr.io/coqui-ai/tts:latest
    container_name: psy-coqui-tts
    restart: unless-stopped
    ports:
      - "${COQUI_HTTP_PORT:-5002}:5002"
    environment:
      - COQUI_TTS_USE_CUDA=${COQUI_TTS_USE_CUDA:-0}
    command:
      - tts-server
      - "--model_name"
      - "${COQUI_MODEL:-tts_models/en/vctk/vits}"
      - "--host"
      - "0.0.0.0"
      - "--port"
      - "5002"
    volumes:
      - coqui_cache:/root/.local/share/tts

volumes:
  ollama_data:
  neo4j_data:
  neo4j_logs:
  neo4j_plugins:
  qdrant_data:
  coqui_cache:

networks:
  default:
    name: psy-ai-stack
COMPOSE
}

install_launcher() {
  common_install_launcher "$STACK_NAME" LAUNCH <<'LAUNCH'
#!/usr/bin/env bash
set -euo pipefail

STACK_DIR="/opt/psyched/docker/ai-stack"
COMPOSE_FILE="${STACK_DIR}/docker-compose.yml"
ENV_FILE="/etc/default/psyched-ai-stack"

if ! command -v docker >/dev/null 2>&1; then
  echo "[ai-stack] docker command not found" >&2
  exit 1
fi

if ! systemctl is-active --quiet docker; then
  systemctl start docker || true
fi

if [ -f "$ENV_FILE" ]; then
  set -o allexport
  # shellcheck disable=SC1090
  . "$ENV_FILE"
  set +o allexport
fi

mkdir -p "$STACK_DIR"

docker compose -f "$COMPOSE_FILE" pull || true

if [ -n "${OLLAMA_MODEL:-}" ]; then
  docker compose -f "$COMPOSE_FILE" run --rm ollama ollama pull "$OLLAMA_MODEL" || true
fi

cleanup() {
  docker compose -f "$COMPOSE_FILE" down --remove-orphans || true
}
trap cleanup EXIT INT TERM

exec docker compose -f "$COMPOSE_FILE" up --remove-orphans
LAUNCH
}

provision() {
  ensure_docker_runtime
  write_env_defaults
  write_compose_file
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
