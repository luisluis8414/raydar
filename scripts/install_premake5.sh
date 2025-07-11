#!/bin/bash

mkdir -p bin

# Detect platform
UNAME_OUT="$(uname -s)"
case "${UNAME_OUT}" in
    Linux*)
        ARCHIVE_NAME="premake-5.0.0-beta2-linux.tar.gz"
        EXE_NAME="premake5"
        ;;
    MINGW*|MSYS*|CYGWIN*)
        ARCHIVE_NAME="premake-5.0.0-beta2-windows.zip"
        EXE_NAME="premake5.exe"
        ;;
    *)
        echo "Unsupported OS: ${UNAME_OUT}"
        exit 1
        ;;
esac

# Download
URL="https://github.com/premake/premake-core/releases/download/v5.0.0-beta2/${ARCHIVE_NAME}"
echo "Downloading $URL"
wget "$URL" || curl -LO "$URL"

# Extract
if [[ "$ARCHIVE_NAME" == *.zip ]]; then
    unzip "$ARCHIVE_NAME"
else
    tar -xzf "$ARCHIVE_NAME"
fi

# Move binary to bin/
mv "$EXE_NAME" "bin/$EXE_NAME"

# Clean up
rm "$ARCHIVE_NAME"
rm -f example.so libluasocket.so

# Make executable (Linux only)
if [[ "$EXE_NAME" == "premake5" ]]; then
    chmod +x "bin/premake5"
fi

echo "Premake installed to ./bin/$EXE_NAME"
