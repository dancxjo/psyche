#!/bin/bash
#
# Bootstrap installer for psyche
# Downloads the repository zip from GitHub (main), extracts into /opt/psyched,
# and runs `tools/provision/bootstrap.sh` from that extracted tree.
# Usage: curl -fsSL https://dancxjo.github.io/psyched | sudo bash
#
set -euo pipefail

REPO_OWNER="dancxjo"
REPO_NAME="psyche"
BRANCH="main"
DEST_DIR="/opt/psyched"

ZIP_URL="https://github.com/${REPO_OWNER}/${REPO_NAME}/archive/refs/heads/${BRANCH}.zip"

echo "Downloading ${REPO_OWNER}/${REPO_NAME}@${BRANCH}..."

# Temporary working directory
TMPDIR=$(mktemp -d)
ZIPPATH="${TMPDIR}/${REPO_NAME}.zip"

cleanup() {
    rm -rf "${TMPDIR}"
}
trap cleanup EXIT

download() {
    if command -v curl >/dev/null 2>&1; then
        curl -fSL -o "${ZIPPATH}" "${ZIP_URL}"
    elif command -v wget >/dev/null 2>&1; then
        wget -q -O "${ZIPPATH}" "${ZIP_URL}"
    else
        echo "Error: Neither curl nor wget is available. Please install one of them first." >&2
        exit 1
    fi
}

extract() {
    # Extract into staging dir for atomic swap
    staged="${DEST_DIR}.new"
    rm -rf "${staged}"
    mkdir -p "${staged}"

    # Extract safely: prefer unzip, then bsdtar/tar
    if command -v unzip >/dev/null 2>&1; then
        unzip -q "${ZIPPATH}" -d "${TMPDIR}"
    elif command -v bsdtar >/dev/null 2>&1; then
        bsdtar -xf "${ZIPPATH}" -C "${TMPDIR}"
    elif command -v tar >/dev/null 2>&1; then
        if tar --version >/dev/null 2>&1 && tar --help | grep -q unzip; then
            tar -xf "${ZIPPATH}" -C "${TMPDIR}"
        else
            echo "Error: no suitable extractor found (requires unzip or bsdtar)." >&2
            exit 1
        fi
    else
        echo "Error: no suitable extractor found (requires unzip or bsdtar)." >&2
        exit 1
    fi

    TOPDIR=$(find "${TMPDIR}" -maxdepth 1 -mindepth 1 -type d | head -n1)
    if [ -z "${TOPDIR}" ]; then
        echo "Error: extracted archive does not contain expected contents." >&2
        exit 1
    fi

    cp -a "${TOPDIR}/." "${staged}/"

    # Swap into place atomically
    if [ -d "${DEST_DIR}" ]; then
        rm -rf "${DEST_DIR}.bak"
        mv "${DEST_DIR}" "${DEST_DIR}.bak"
    fi
    mv "${staged}" "${DEST_DIR}"
}

ensure_unzip() {
    if command -v unzip >/dev/null 2>&1; then
        return 0
    fi

    echo "'unzip' not found; attempting to install via apt-get (requires sudo)"
    if ! command -v sudo >/dev/null 2>&1; then
        echo "Error: 'sudo' is required to install unzip but was not found." >&2
        exit 1
    fi

    sudo apt-get update -y && sudo apt-get install -y unzip || {
        echo "Error: failed to install 'unzip' via apt-get" >&2
        exit 1
    }
}

run_bootstrap() {
    BOOTSTRAP="${DEST_DIR}/tools/provision/bootstrap.sh"
    if [ ! -x "${BOOTSTRAP}" ]; then
        if [ -f "${BOOTSTRAP}" ]; then
            chmod +x "${BOOTSTRAP}"
        else
            echo "Error: bootstrap script not found at ${BOOTSTRAP}" >&2
            exit 1
        fi
    fi

    echo "Running bootstrap script from ${BOOTSTRAP}..."
    # Run the bootstrap script with bash from the repository root
    (cd "${DEST_DIR}" && bash "${BOOTSTRAP}")
}

download
ensure_unzip
extract
run_bootstrap

echo "Install finished."