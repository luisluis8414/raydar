# VoxVision

A C++ project for voxel-based graphics processing.

## Quick Start

### Prerequisites
- C++ compiler (GCC/Clang)
- GNU Make
- wget or curl (for dependency installation)

### Setup

1. Install Premake5:
```bash
# Make the install script executable
chmod +x scripts/install_premake5.sh

# Run the installation script
./scripts/install_premake5.sh
```

2. Install dependencies:
```bash
# Install JSON library
./scripts/install_json.sh
```

### Build and Run

You can use the run script to build and run the project:
```bash
# Build and run Debug version
./scripts/run.sh Debug

# Build and run Release version
./scripts/run.sh Release
```

Alternatively, you can build and run manually:

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

3. Run:
```bash
# Debug version
./bin/Debug/Playground

# Release version
./bin/Release/Playground
```

## Project Structure
```
.
├── pixelToVoxel/    # Core library
│   ├── include/     # Public headers
│   ├── src/         # Implementation
│   └── tests/       # Test files
├── Playground/      # Example application
├── docs/           # Documentation
│   └── metadata_format.md  # Metadata file format specification
└── scripts/         # Build and setup scripts
```

## Dependencies
- [nlohmann/json](https://github.com/nlohmann/json) - JSON for Modern C++

## Documentation

- [Metadata File Format](docs/metadata_format.md) - Specification for camera metadata JSON files
