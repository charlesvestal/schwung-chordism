#!/bin/bash
# Install Chordism module to Move
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

if [ ! -d "dist/chordism" ]; then
    echo "Error: dist/chordism not found. Run ./scripts/build.sh first."
    exit 1
fi

echo "=== Installing Chordism Module ==="
ssh ableton@move.local "mkdir -p /data/UserData/schwung/modules/sound_generators/chordism"
scp -r dist/chordism/* ableton@move.local:/data/UserData/schwung/modules/sound_generators/chordism/

ssh ableton@move.local "chmod -R a+rw /data/UserData/schwung/modules/sound_generators/chordism"

echo "=== Install Complete ==="
echo "Restart Schwung to load the new module."
