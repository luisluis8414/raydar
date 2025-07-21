#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace raydar {

/**
 * @brief Contains metadata about a single camera frame
 */
struct FrameInfo {
    int camera_index;                 ///< Unique identifier for the camera
    int frame_index;                  ///< Sequential frame number in the camera's sequence
    Eigen::Vector3f camera_rotation;  ///< Camera rotation in degrees (XYZ Euler angles)
    float FOV_deg;                    ///< Field of view in degrees
    std::string image_file;           ///< Path to the corresponding image file
    Eigen::Vector3f camera_position;  ///< Camera position in world space coordinates
};

/**
 * @brief Represents a grayscale image with floating-point pixel values
 */
struct Image {
    int width;                  ///< Width of the image in pixels
    int height;                 ///< Height of the image in pixels
    std::vector<float> pixels;  ///< Pixel values stored in row-major order
};

/**
 * @brief Represents a pixel location where motion was detected
 */
struct PixelChange {
    int x;         ///< X coordinate of the pixel
    int y;         ///< Y coordinate of the pixel
    float change;  ///< Magnitude of the change in pixel value

    // comparison operator for use in std::set for motion_detection()
    bool operator<(const PixelChange& other) const {
        return (x < other.x) || (x == other.x && y < other.y);
    }
};

/**
 * @brief Contains motion detection results for an image pair
 */
struct DetectionArray {
    int width;                                    ///< Width of the analyzed image
    int height;                                   ///< Height of the analyzed image
    std::vector<PixelChange> pixels_with_motion;  ///< List of pixels where motion was detected
};

struct SkewResult {
    bool valid;  ///< indicates whether the calculation was successful (rays are not parallel or
    ///< invalid)
    float distance;            ///< the shortest distance between the two rays
    Eigen::Vector3f midpoint;  ///< the midpoint of the shortest line connecting the two rays at
    ///< their closest point
};

struct Ray {
    int camera_id;                    ///< the camera that generated this ray
    Eigen::Vector3f ray;              ///< the direction vector of the ray in world coordinates
    Eigen::Vector3f camera_position;  ///< the position of the camera in world coordinates
};

struct RaysAtFrame {
    int frame_id;           ///< the frame index (timestamp)
    std::vector<Ray> rays;  ///< all rays from all cameras for this timestamp
};

/**
 * @brief Detects objects in 3D space from camera metadata and image data
 *
 * @param metadata_file_path Path to the JSON metadata file
 * @param detect_motion_threshold Threshold value for motion detection between frames
 * @param min_distance Minimum distance between detected motion objects
 *
 * @details This function processes the camera metadata and corresponding images to:
 *          1. Detect motion between consecutive frames
 *          2. Find centers of motion objects
 *          3. Generate rays from camera positions through motion centers
 *          4. Create visualization output for debugging
 */
void detect_objects(const std::string& metadata_file_path, float detect_motion_threshold,
                    float min_distance = 5.0f);

}  // namespace raydar