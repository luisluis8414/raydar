#!/bin/bash

# Function to print usage
print_usage() {
    echo "Usage: $0 [Debug|Release]"
    echo "Example: $0 Debug"
}

# Check if configuration argument is provided
if [ $# -ne 1 ]; then
    echo "Error: Configuration argument required"
    print_usage
    exit 1
fi

# Convert input to lowercase for comparison
CONFIG_LOWER=$(echo "$1" | tr '[:upper:]' '[:lower:]')

# Validate configuration argument
if [ "$CONFIG_LOWER" != "debug" ] && [ "$CONFIG_LOWER" != "release" ]; then
    echo "Error: Invalid configuration. Must be either 'Debug' or 'Release'"
    print_usage
    exit 1
fi

# Use the original case for the configuration
CONFIG=$1

echo "Building project in $CONFIG configuration..."

# Generate build files if they don't exist
if [ ! -f "Makefile" ]; then
    echo "Generating build files..."
    ./bin/premake5 gmake2
    if [ $? -ne 0 ]; then
        echo "Error: Failed to generate build files"
        exit 1
    fi
fi

# Build the project
echo "Building..."
make config=$CONFIG_LOWER
if [ $? -ne 0 ]; then
    echo "Error: Build failed"
    exit 1
fi

# Run the program
echo "Running Playground..."
./bin/$CONFIG/Playground
if [ $? -ne 0 ]; then
    echo "Error: Program execution failed"
    exit 1
fi 