#!/bin/bash
set -e

# Default path assumes the container workspace structure
LIB_PATH=${1:-/home/workspace/src/pfields_2025/pfield_library}
BUILD_DIR="/tmp/pfield_library_build"

if [ ! -d "$LIB_PATH" ]; then
    echo "Error: pfield_library source not found at $LIB_PATH"
    echo "Usage: build_pfield_lib.sh [path_to_pfield_library]"
    exit 1
fi

echo "Building pfield_library from $LIB_PATH..."
echo "Using temporary build directory: $BUILD_DIR"

# Clean previous build to ensure freshness
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

# Configure and install to /usr/local (default system path in container)
# We use -S (source) and -B (build) to keep the source directory clean
cmake -S "$LIB_PATH" -B "$BUILD_DIR" -DCMAKE_INSTALL_PREFIX=/usr/local

# Build
cmake --build "$BUILD_DIR" -j$(nproc)

# Install
cmake --install "$BUILD_DIR"

# Update shared library cache
ldconfig

echo "pfield_library successfully installed."
