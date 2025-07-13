#pragma once
#include "PixelToVoxel.hpp"

namespace ptv {

/**
 * @brief Visualizes motion detection by creating an image with red highlights on motion pixels
 * 
 * @param curr_img Current image
 * @param detection Detection results from detectMotion
 * @param output_path Path to save the output PNG file
 */
void visualizeMotion(const Image& curr_img, const DetectionArray& detection, const std::string& output_path);

} // namespace ptv 