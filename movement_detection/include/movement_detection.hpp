#pragma once
#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>

namespace ptv {

    /**
     * @brief A 3D vector structure representing coordinates or directions in 3D space
    //  */
    // struct Vec3 {
    //     float X;  ///< X component of the vector
    //     float Y;  ///< Y component of the vector
    //     float Z;  ///< Z component of the vector
    // };

    /**
     * @brief Contains metadata about a single camera frame
     */
    struct FrameInfo {
        int camera_index;      ///< Unique identifier for the camera
        int frame_index;       ///< Sequential frame number in the camera's sequence
        Eigen::Vector3f camera_rotation;  ///< Camera rotation in degrees (XYZ Euler angles)
        float fov_degrees;     ///< Field of view in degrees
        std::string image_file; ///< Path to the corresponding image file
        Eigen::Vector3f camera_position;  ///< Camera position in world space coordinates
    };

    /**
     * @brief Represents a grayscale image with floating-point pixel values
     */
    struct Image {
        int width;             ///< Width of the image in pixels
        int height;            ///< Height of the image in pixels
        std::vector<float> pixels; ///< Pixel values stored in row-major order
    };

    /**
     * @brief Represents a pixel location where motion was detected
     */
    struct PixelChange {
        int x;      ///< X coordinate of the pixel
        int y;      ///< Y coordinate of the pixel
        float change; ///< Magnitude of the change in pixel value
    };

    /**
     * @brief Contains motion detection results for an image pair
     */
    struct DetectionArray {
        int width;   ///< Width of the analyzed image
        int height;  ///< Height of the analyzed image
        std::vector<PixelChange> pixels_with_motion; ///< List of pixels where motion was detected
    };

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

    /**
     * @brief Detects motion between two consecutive frames
     *
     * @param prev_img Previous frame image
     * @param curr_img Current frame image
     * @param motion_threshold Minimum difference in pixel values to consider as motion
     * @return DetectionArray containing the detected motion pixels and their changes
     *
     * @details This function compares corresponding pixels between two consecutive frames
     *          and identifies locations where the absolute difference in pixel values
     *          exceeds the specified threshold.
     */
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
     * @param detect_motion_threshold Threshold value for motion detection between frames
     *
     * @details This function processes the camera metadata and corresponding images to:
     *          1. Detect motion between consecutive frames
     *          2. Find centers of motion objects
     *          3. Generate rays from camera positions through motion centers
     *          4. Create visualization output for debugging
     */
    void generate_voxel_grid(const std::string& metadata_file_path, const float detect_motion_threshold);

} // namespace ptv 