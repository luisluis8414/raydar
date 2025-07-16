#pragma once

#include <Eigen/Dense>
#include "movement_detection.hpp"

namespace raydar {

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

 
    /**
     * @brief Applies Euler angle rotation to a vector
     *
     * @param v Input vector to rotate
     * @param euler_deg Euler angles in degrees (XYZ order)
     * @return Vec3 Rotated vector
     */
    Eigen::Vector3f apply_rotation(const Eigen::Vector3f& v, const Eigen::Vector3f& euler_deg);

    /**
     * @brief Calculates the ray direction from camera through a pixel
     *
     * @param info Camera frame information containing position, rotation and FOV
     * @param pixel_x X coordinate of the pixel in image space
     * @param pixel_y Y coordinate of the pixel in image space
     * @param img_width Width of the image in pixels
     * @param img_height Height of the image in pixels
     * @return Vec3 Normalized direction vector from camera through the pixel
     */
    Eigen::Vector3f get_ray_direction(const FrameInfo& info, int pixel_x, int pixel_y, int img_width, int img_height);

} // namespace raydar 