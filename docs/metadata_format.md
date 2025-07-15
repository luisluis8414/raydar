# Metadata File Format

## Overview

The metadata file contains camera and frame information used by the movement_detection library. It should be provided as a JSON file containing an array of frame metadata objects.

## Format Specification

```json
[
    {
        "camera_index": number,     // Index of the camera (integer)
        "frame_index": number,      // Sequential frame number (integer)
        "camera_rotation": {        // Camera rotation in XYZ Euler angles (degrees)
            "X": number,           // First rotation around X-axis
            "Y": number,           // Second rotation around Y-axis
            "Z": number            // Third rotation around Z-axis
        },
        "fov_degrees": number,      // Field of view in degrees (float)
        "image_file": string,       // Path to the corresponding image file
        "camera_position": {        // Camera position in world space
            "X": number,           // X coordinate (meters)
            "Y": number,           // Y coordinate (meters)
            "Z": number            // Z coordinate (meters)
        }
    },
    // ... Additional frames ...
]
```

## Example

```json
[
  {
    "camera_index": 0,
    "frame_index": 1,
    "camera_rotation": {
      "X": 89.999995674289,
      "Y": 149.99999734394112,
      "Z": 1.3184902819141923e-5
    },
    "fov_degrees": 112.61986782788955,
    "image_file": "camera0_frame0001.png",
    "camera_position": {
      "X": 75.0,
      "Y": 0.0,
      "Z": 2.0
    }
  }
]
```

## Field Descriptions

### Required Fields

- `camera_index`: Integer identifying the camera that captured this frame
- `frame_index`: Sequential number of the frame in the capture sequence
- `camera_rotation`: Object containing rotation angles in XYZ Euler order
  - `X`: First rotation around X-axis (intrinsic rotation)
  - `Y`: Second rotation around Y-axis (intrinsic rotation)
  - `Z`: Third rotation around Z-axis (intrinsic rotation)
- `fov_degrees`: Camera's field of view in degrees
- `image_file`: Relative or absolute path to the image file
- `camera_position`: Object containing camera position coordinates in meters
  - `X`: Position on X-axis
  - `Y`: Position on Y-axis
  - `Z`: Position on Z-axis

### Data Types

- All numerical values should be provided as numbers (integers or floating-point)
- Text values should be provided as strings
- Coordinate and rotation values are typically floating-point numbers
- Indices (camera_index, frame_index) are integers

## Usage

Place your metadata file in the `data` directory. The file can then be loaded using:

```cpp
ptv::movement_detection converter;
std::vector<ptv::FrameInfo> frames = converter.loadMetadata("data/metadata.json");
```

## Validation

The library will validate the JSON structure when loading. Any errors in the format will result in:

1. Error messages printed to stderr
2. An empty vector returned from the loadMetadata function

## Notes

- Rotations use XYZ Euler angles (intrinsic rotations):
  1. First rotation around X-axis
  2. Second rotation around Y-axis
  3. Third rotation around Z-axis
- All rotations are in degrees (not radians)
- All positions are in meters
- Coordinate system is right-handed
- File paths in `image_file` can be relative to the working directory or absolute paths
