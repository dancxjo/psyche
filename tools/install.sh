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
    # Ensure destination exists
    mkdir -p "${DEST_DIR}"

    # Extract safely: prefer unzip, then bsdtar/tar
    if command -v unzip >/dev/null 2>&1; then
        unzip -q "${ZIPPATH}" -d "${TMPDIR}"
    elif command -v bsdtar >/dev/null 2>&1; then
        bsdtar -xf "${ZIPPATH}" -C "${TMPDIR}"
    elif command -v tar >/dev/null 2>&1; then
        # GNU tar can handle zip with --use-compress-program but to be safer try busybox unzip
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

    # The zip contains a top-level directory like psyche-main; move its contents into DEST_DIR
    TOPDIR=$(find "${TMPDIR}" -maxdepth 1 -mindepth 1 -type d | head -n1)
    if [ -z "${TOPDIR}" ]; then
        echo "Error: extracted archive does not contain expected contents." >&2
        exit 1
    fi

    # Copy contents into DEST_DIR (overwrite but preserve ownership as root)
    cp -a "${TOPDIR}/." "${DEST_DIR}/"
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
extract
run_bootstrap

echo "Install finished."