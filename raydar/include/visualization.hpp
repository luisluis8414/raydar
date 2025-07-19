#pragma once

#include <string>
#include <utility>
#include <vector>

#include "movement_detection.hpp"

namespace raydar {

/**
 * @brief Visualizes motion detection by creating an image with red highlights on motion pixels
 *        and green highlights on the center pixels of detected objects.
 *
 * @param curr_img Current image
 * @param detection Detection results from detectMotion
 * @param output_path Path to save the output PNG file
 * @param centers Vector of (x, y) pairs for object centers to highlight in green
 */
void visualize_motion(const Image& curr_img, const DetectionArray& detection,
                      const std::string& output_path,
                      const std::vector<std::pair<int, int>>& centers);

void visualize_flight_path(const Image& base_img,
                           const std::vector<std::pair<int, int>>& all_centers,
                           const std::string& output_path);
}  // namespace raydar