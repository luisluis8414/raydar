#pragma once

#include "PixelToVoxel.hpp"

namespace ptv {

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief Visualizes motion detection by creating an image with red highlights on motion pixels
 * 
 * @param curr_img Current image
 * @param detection Detection results from detectMotion
 * @param output_path Path to save the output PNG file
 */
void visualizeMotion(const Image& curr_img, const DetectionArray& detection, const std::string& output_path);

// Vector math helpers
Vec3 normalize(const Vec3& v);
Vec3 applyRotation(const Vec3& v, const Vec3& euler_deg);

// Ray casting functions
Vec3 getRayDirection(const FrameInfo& info, int pixel_x, int pixel_y, int img_width, int img_height);
void traceRayThroughVoxels(std::vector<float>& voxel_grid, const Vec3& origin, const Vec3& dir, float increment_value);

} // namespace ptv 