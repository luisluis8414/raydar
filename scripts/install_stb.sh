#!/bin/bash

# Create directories if they don't exist
mkdir -p deps/stb

# Define the stb_image.h file URL
STB_IMAGE_URL="https://github.com/nothings/stb/raw/master/stb_image.h"

echo "Downloading stb_image.h..."
if command -v curl &> /dev/null; then
    curl -L $STB_IMAGE_URL -o deps/stb/stb_image.h
elif command -v wget &> /dev/null; then
    wget $STB_IMAGE_URL -O deps/stb/stb_image.h
else
    echo "Error: Neither curl nor wget is installed. Please install one of them."
    exit 1
fi

if [ $? -eq 0 ]; then
    echo "Successfully installed stb_image.h to deps/stb/"
else
    echo "Failed to download stb_image.h"
    exit 1
fi 