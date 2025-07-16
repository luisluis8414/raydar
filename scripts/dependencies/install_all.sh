#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Install stb headers
echo "Installing stb headers..."
bash "$SCRIPT_DIR/install_stb.sh"

echo "Installing nlohmann/json..."
bash "$SCRIPT_DIR/install_nlohmann_json.sh"

echo "Installing eigen..."
bash "$SCRIPT_DIR/install_eigen.sh"

echo "All dependencies installed successfully."
