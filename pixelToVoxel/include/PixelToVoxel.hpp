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
struct Image {
    int width;
    int height;
    std::vector<float> pixels; 
};
struct PixelChange {
    int x;
    int y;
    float change;
};

struct DetectionArray {
    int width;
    int height;
    std::vector<PixelChange> pixels_with_motion;
};

const int VOXEL_GRID_N = 500;                            // Number of voxels per dimension (N x N x N)
const float VOXEL_SIZE = 5.0f;                           // Size of each voxel in meters (total span = RESOLUTION * SIZE)
const Vec3 GRID_CENTER = {0.f, 0.f, 1250.f};    // Center of the grid in world coordinates (x, y, z in meters)

// Motion Detection and Accumulation Settings
const float MOTION_THRESHOLD = 2.0f;    // Pixel difference threshold for detecting motion (higher = less sensitive)

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
std::map<int, std::vector<FrameInfo>> load_metadata(const std::string& metadata_file);

/**
 * @brief Loads an grayscaleimage file and converts it to float values
 * 
 * @param path Path to the image file
 * @param out Output Image structure to fill with loaded data
 */
void load_image(const std::string& path, Image& out);

DetectionArray detect_motion(const Image& prev_img, const Image& curr_img, float motion_threshold);

/**
 * @brief Finds the center pixels of connected motion objects in a detection array.
 *
 * This function takes a `DetectionArray` containing pixels where motion was detected
 * and groups them into connected components (objects) based on 8-connected adjacency.
 * For each connected component, it calculates the center pixel (geometric centroid)
 * and returns a list of these centers.
 *
 * The adjacency is defined as 8-connected, meaning pixels are considered neighbors
 * if they share an edge or corner (Manhattan or Chebyshev distance ≤ 1).
 *
 * @param da The `DetectionArray` containing the width, height, and a list of pixels
 *           with motion (`PixelChange` objects).
 * @return A `std::vector<std::pair<int, int>>` containing the center pixels of each
 *         detected motion object. Each pair represents the (x, y) coordinates of a center.
 *
 * @note The center is calculated as the rounded average of the x and y coordinates
 *       of all pixels in a connected component. If weighting by `change` is needed,
 *       the function can be modified to compute a weighted centroid.
 */
 std::vector<std::pair<int, int>> find_object_centers(const DetectionArray& da);

/**
 * @brief Generates a voxel grid from camera metadata and image data
 * 
 * @param metadata_file_path Path to the JSON metadata file
 */
void generate_voxel_grid(const std::string& metadata_file_path);

} // namespace ptv 