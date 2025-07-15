#pragma once

#include "PixelToVoxel.hpp"

namespace ptv {

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief Visualizes motion detection by creating an image with red highlights on motion pixels
 *        and green highlights on the center pixels of detected objects.
 * 
 * @param curr_img Current image
 * @param detection Detection results from detectMotion
 * @param output_path Path to save the output PNG file
 * @param centers Vector of (x, y) pairs for object centers to highlight in green
 */
void visualize_motion(const Image& curr_img, const DetectionArray& detection, const std::string& output_path, const std::vector<std::pair<int, int>>& centers);

// Vector math helpers
Vec3 normalize(const Vec3& v);
Vec3 apply_rotation(const Vec3& v, const Vec3& euler_deg);

// Ray casting functions
Vec3 get_ray_direction(const FrameInfo& info, int pixel_x, int pixel_y, int img_width, int img_height);
std::vector<Vec3> find_voxel_cluster_centroids(const std::vector<int>& camera_count, int min_count, const Vec3& grid_min, float voxel_size, int N);

// puts all past positions in the last picture of a camera series (flightpath)
void visualize_flight_path(const Image& base_img, const std::vector<std::pair<int, int>>& all_centers, const std::string& output_path);
} // namespace ptv 