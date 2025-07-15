#pragma once

#include "movement_detection.hpp"

namespace ptv {

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

    /**
     * @brief Normalizes a 3D vector to unit length
     *
     * @param v Input vector to normalize
     * @return Vec3 Normalized vector with length 1
     */
    Vec3 normalize(const Vec3& v);

    /**
     * @brief Applies Euler angle rotation to a vector
     *
     * @param v Input vector to rotate
     * @param euler_deg Euler angles in degrees (XYZ order)
     * @return Vec3 Rotated vector
     */
    Vec3 apply_rotation(const Vec3& v, const Vec3& euler_deg);

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
    Vec3 get_ray_direction(const FrameInfo& info, int pixel_x, int pixel_y, int img_width, int img_height);

} // namespace ptv 