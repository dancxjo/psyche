#!/usr/bin/env bash
set -euo pipefail

if [ $# -ne 1 ]; then
  echo "Usage: $0 /path/to/wheel.whl" >&2
  exit 2
fi

SRC=$1
if [ ! -f "$SRC" ]; then
  echo "Wheel not found: $SRC" >&2
  exit 3
fi

DEST_DIR="$(dirname "$0")/../wheels"
mkdir -p "$DEST_DIR"

SIZE=$(stat -c%s "$SRC")
if [ $SIZE -gt $((50*1024*1024)) ]; then
  echo "Warning: wheel is larger than 50MB ($((SIZE/1024/1024)) MB). Consider using Git LFS." >&2
fi

cp "$SRC" "$DEST_DIR/"
WNAME=$(basename "$SRC")
git add "$DEST_DIR/$WNAME"
git commit -m "Add wheel: $WNAME" || echo "Commit failed (maybe running in a detached head). Please commit manually."
echo "Wheel copied to $DEST_DIR/$WNAME"
