#pragma once
#include <string>
#include <vector>
#include <map>

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

struct Image
{
    int width;
    int height;
    std::vector<float> pixels; 
};


const int VOXEL_GRID_N = 500;          // Number of voxels per dimension (N x N x N)
const float VOXEL_SIZE = 5.0f;                  // Size of each voxel in meters (total span = RESOLUTION * SIZE)
const Vec3 GRID_CENTER = {0.f, 0.f, 1250.f};    // Center of the grid in world coordinates (x, y, z in meters)

// Motion Detection and Accumulation Settings
const float MOTION_THRESHOLD = 2.0f;    // Pixel difference threshold for detecting motion (higher = less sensitive)
// const float ATTENUATION_ALPHA = 0.1f;   // Distance-based attenuation factor for ray accumulation (higher = more falloff)

class PixelToVoxel {
private:
    /**
     * @brief Loads camera metadata from a JSON file and groups frames by camera index
     * 
     * @param metadata_file Path to the JSON file containing camera metadata
     * @return std::map<int, std::vector<FrameInfo>> Map of camera indices to their respective frames
     * 
     * @details The JSON file should contain an array of frame metadata, where each frame has:
     *          - camera_index: Index of the camera
     *          - frame_index: Sequential frame number
     *          - camera_rotation: {X, Y, Z} rotation in degrees (XYZ Euler angles)
     *          - fov_degrees: Field of view in degrees
     *          - image_file: Path to the corresponding image file
     *          - camera_position: {X, Y, Z} position in world space
     * 
     * @throws None - Errors are handled internally 
     */
     std::map<int, std::vector<FrameInfo>> loadMetadata(const std::string& metadata_file);

     void load_image(const std::string path, Image& out);
public:
    PixelToVoxel();
    ~PixelToVoxel();


    void generateVoxelGrid(const std::string& metadata_file_path);
};

} // namespace ptv 