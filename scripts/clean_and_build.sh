#!/bin/bash

set -e

# Remove build directories and files
echo "Cleaning build directories and files..."
rm -rf bin/Debug bin/Release obj Makefile compile_commands.json

if [ ! -f "Makefile" ]; then
    echo "Generating build files..."
    ./bin/premake5 gmake2
    if [ $? -ne 0 ]; then
        echo "Error: Failed to generate build files"
        exit 1
    fi
fi

# Run bear with make clean and make
echo "Running 'bear -- make clean'..."
bear -- make clean

echo "Running 'bear -- make'..."
bear -- make

echo "Clean and build complete." 