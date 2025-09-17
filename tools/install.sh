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
    # Detect archive type and extract appropriately.
    # Prefer: unzip (zip), python3 zipfile (zip), bsdtar (many), tar (tarballs).
    MIMETYPE=""
    if command -v file >/dev/null 2>&1; then
        MIMETYPE=$(file --brief --mime-type "${ZIPPATH}" 2>/dev/null || true)
    fi

    case "${MIMETYPE}" in
        application/zip|application/x-zip|application/x-zip-compressed)
            if command -v unzip >/dev/null 2>&1; then
                unzip -q "${ZIPPATH}" -d "${TMPDIR}"
            elif command -v python3 >/dev/null 2>&1; then
                echo "[install] 'unzip' not available; using python3 zipfile to extract"
                python3 - <<PY
import sys, zipfile
from pathlib import Path
zf = zipfile.ZipFile(r'${ZIPPATH}')
zf.extractall(r'${TMPDIR}')
zf.close()
PY
            elif command -v bsdtar >/dev/null 2>&1; then
                bsdtar -xf "${ZIPPATH}" -C "${TMPDIR}"
            else
                echo "Error: no tool available to extract ZIP archives (install 'unzip' or ensure python3 is present)." >&2
                exit 1
            fi
            ;;
        application/x-gzip|application/x-tar|application/x-xz|application/x-bzip2)
            # tar-compatible archive
            if command -v tar >/dev/null 2>&1; then
                tar -xf "${ZIPPATH}" -C "${TMPDIR}"
            elif command -v bsdtar >/dev/null 2>&1; then
                bsdtar -xf "${ZIPPATH}" -C "${TMPDIR}"
            else
                echo "Error: no tar-compatible extractor found." >&2
                exit 1
            fi
            ;;
        *)
            # Unknown mime type: try unzip, then bsdtar, then tar as last resort
            if command -v unzip >/dev/null 2>&1; then
                unzip -q "${ZIPPATH}" -d "${TMPDIR}" || true
            elif command -v bsdtar >/dev/null 2>&1; then
                bsdtar -xf "${ZIPPATH}" -C "${TMPDIR}" || true
            elif command -v tar >/dev/null 2>&1; then
                tar -xf "${ZIPPATH}" -C "${TMPDIR}" || true
            else
                echo "Error: no extractor available for archive." >&2
                exit 1
            fi
            ;;
    esac

    TOPDIR=$(find "${TMPDIR}" -maxdepth 1 -mindepth 1 -type d | head -n1)
    if [ -z "${TOPDIR}" ]; then
        echo "Error: extracted archive does not contain expected contents." >&2
        exit 1
    fi

    cp -a "${TOPDIR}/." "${staged}/"

    # Preserve existing workspace to keep build artifacts/cache
    if [ -d "${DEST_DIR}/ws" ]; then
        echo "[install] Preserving existing workspace: ${DEST_DIR}/ws"
        rm -rf "${staged}/ws" 2>/dev/null || true
        cp -a "${DEST_DIR}/ws" "${staged}/" || true
    fi

    # Swap into place atomically
    if [ -d "${DEST_DIR}" ]; then
        rm -rf "${DEST_DIR}.bak"
        mv "${DEST_DIR}" "${DEST_DIR}.bak"
    fi
    mv "${staged}" "${DEST_DIR}"
}

ensure_unzip() {
    # Ensure both unzip and zip are available so we can extract and create zips
    if command -v unzip >/dev/null 2>&1 && command -v zip >/dev/null 2>&1; then
        return 0
    fi

    echo "'unzip' or 'zip' not found; attempting to install via apt-get (requires sudo)"
    if ! command -v sudo >/dev/null 2>&1; then
        echo "Error: 'sudo' is required to install unzip/zip but was not found." >&2
        exit 1
    fi

    # Install common extraction helpers: unzip, zip, libarchive-tools (bsdtar), and file
    sudo apt-get update -y && sudo apt-get install -y unzip zip libarchive-tools file || {
        echo "Error: failed to install unzip/zip (or helpers) via apt-get" >&2
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

    echo "Running bootstrap script from ${BOOTSTRAP} (apply mode)..."
    # Run the bootstrap script with bash from the repository root in apply mode
    (cd "${DEST_DIR}" && bash "${BOOTSTRAP}" --apply)
}

download
ensure_unzip
extract
run_bootstrap

echo "Install finished."