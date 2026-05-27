#!/usr/bin/env bash
# Build Chordism module for Schwung (ARM64)
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
IMAGE_NAME="schwung-chordism-builder"

if [ -z "$CROSS_PREFIX" ] && [ ! -f "/.dockerenv" ]; then
    echo "=== Chordism Module Build (via Docker) ==="
    if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
        echo "Building Docker image (first time only)..."
        docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$REPO_ROOT"
    fi
    echo "Running build..."
    docker run --rm \
        -v "$REPO_ROOT:/build" \
        -u "$(id -u):$(id -g)" \
        -w /build \
        "$IMAGE_NAME" \
        ./scripts/build.sh
    echo "=== Done ==="
    exit 0
fi

CROSS_PREFIX="${CROSS_PREFIX:-aarch64-linux-gnu-}"
cd "$REPO_ROOT"

echo "=== Building Chordism Module ==="
mkdir -p build
mkdir -p dist/chordism

echo "Compiling DSP plugin..."
${CROSS_PREFIX}g++ -g -O3 -shared -fPIC -std=c++14 \
    src/dsp/chordism_plugin.cpp \
    -o build/dsp.so \
    -lm

echo "Packaging..."
cat src/module.json > dist/chordism/module.json
[ -f LICENSE ] && cat LICENSE > dist/chordism/LICENSE
cat build/dsp.so > dist/chordism/dsp.so
chmod +x dist/chordism/dsp.so

cd dist
tar -czvf chordism-module.tar.gz chordism/
cd ..

echo "=== Build Complete ==="
echo "Output: dist/chordism/"
echo "Tarball: dist/chordism-module.tar.gz"
