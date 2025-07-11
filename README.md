# VoxVision

A C++ project for voxel-based graphics processing.

## Quick Start

### Prerequisites
- C++ compiler (GCC/Clang)
- GNU Make
- wget or curl (for premake installation)

### Setup

1. Install Premake5:
```bash
# Make the install script executable
chmod +x scripts/install_premake5.sh

# Run the installation script
./scripts/install_premake5.sh
```
This will download and set up premake5 in the `bin` directory.

### Build

1. Generate build files:
```bash
./bin/premake5 gmake2
```

2. Compile:
```bash
# Debug build
make config=debug

# Release build
make config=release
```

### Run
```bash
# Debug version
./bin/Debug/HelloWorld

# Release version
./bin/Release/HelloWorld
```

## Project Structure
```
.
├── pixelToVoxel/    # Core library
│   ├── include/     # Public headers
│   ├── src/         # Implementation
│   └── tests/       # Test files
├── HelloWorld/      # Example application
└── scripts/         # Build and setup scripts
```
