# VoxVision

A C++ project for voxel graphics processing.

## Quick Start

### Prerequisites
- C++ compiler (GCC/Clang)
- GNU Make
- wget or curl (for dependency installation)

## ⚠️ Cloning the Repository

### Large Files (Blender Scenes)

This project includes large `.blend` files (e.g., 3D scene with 5 cameras and an f-35) which are tracked using [Git LFS (Large File Storage)](https://git-lfs.github.com/).

Before cloning the repository, make sure Git LFS is installed:

```bash
# One-time setup
git lfs install

# Clone the repository
git clone https://github.com/luisluis8414/vox_vision.git
```

If you've already cloned without LFS, run:

```bash
git lfs pull
```

**Note:** Blender backup files such as `.blend1` and `.blend@` are excluded via `.gitignore`. Only the primary `.blend` files are tracked.


### Setup

1. Install Premake5:
```bash
# Install premake into bin/
./scripts/install_premake5.sh
```

2. Install dependencies:
```bash
# Install JSON library into deps
./scripts/install_nlohmann_json.sh
```

### Build and Run

You can use the run script to build and run the project:
```bash
# Build and run Debug version
./scripts/build_and_run.sh Debug

# Build and run Release version
./scripts/build_and_run.sh Release
```

Alternatively, you can build and run manually:

1. Generate build files:
```bash
# You can replace 'gmake2' with any other Premake action (e.g., vs2022, xcode4)
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
├── deps/           # External dependencies
└── scripts/         # Build and setup scripts
```

## Dependencies
- [nlohmann/json](https://github.com/nlohmann/json) - JSON for Modern C++

## Documentation

- [Metadata File Format](docs/metadata_format.md) - Specification for camera metadata JSON files

