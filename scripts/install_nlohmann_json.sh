#!/bin/bash

# Create directories if they don't exist
mkdir -p deps/nlohmann

# Define the JSON file URL (using the single include version)
JSON_URL="https://github.com/nlohmann/json/raw/develop/single_include/nlohmann/json.hpp"

echo "Downloading nlohmann/json..."
if command -v curl &> /dev/null; then
    curl -L $JSON_URL -o deps/nlohmann/json.hpp
elif command -v wget &> /dev/null; then
    wget $JSON_URL -O deps/nlohmann/json.hpp
else
    echo "Error: Neither curl nor wget is installed. Please install one of them."
    exit 1
fi

if [ $? -eq 0 ]; then
    echo "Successfully installed json.hpp to deps/nlohmann/"
else
    echo "Failed to download json.hpp"
    exit 1
fi 