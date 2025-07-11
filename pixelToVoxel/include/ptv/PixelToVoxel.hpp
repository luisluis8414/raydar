#pragma once
#include <string>
#include <vector>

namespace ptv {

struct Vec3 {
    float X;
    float Y;
    float Z;
};

struct FrameInfo {
    int camera_index;
    int frame_index;
    Vec3 camera_rotation;
    float fov_degrees;
    std::string image_file;
    Vec3 camera_position;
};

class PixelToVoxel {
public:
    PixelToVoxel();
    ~PixelToVoxel();

    /**
     * @brief Loads camera metadata from a JSON file
     * 
     * @param metadata_file Path to the JSON file containing camera metadata
     * @return std::vector<FrameInfo> Vector of frame information, empty if loading fails
     * 
     * @details The JSON file should contain an array of frame metadata, where each frame has:
     *          - camera_index: Index of the camera
     *          - frame_index: Sequential frame number
     *          - camera_rotation: {X, Y, Z} rotation in degrees
     *          - fov_degrees: Field of view in degrees
     *          - image_file: Path to the corresponding image file
     *          - camera_position: {X, Y, Z} position in world space
     * 
     * @throws None - Errors are handled internally and reported to stderr
     */
    std::vector<FrameInfo> loadMetadata(const std::string& metadata_file);

    
    bool convertPixelToVoxel();
};

} // namespace ptv 