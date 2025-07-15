#include "visualization.hpp"
#include <cmath>
#include <vector>

// Add stb_image_write
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

namespace ptv {

    void visualize_motion(const Image& curr_img, const DetectionArray& detection, const std::string& output_path, const std::vector<std::pair<int, int>>& centers) {
        std::vector<unsigned char> rgb_data(curr_img.width * curr_img.height * 3);

        // Fill with grayscale
        for (int y = 0; y < curr_img.height; y++) {
            for (int x = 0; x < curr_img.width; x++) {
                unsigned char val = static_cast<unsigned char>(curr_img.pixels[y * curr_img.width + x]);
                int idx = (y * curr_img.width + x) * 3;
                rgb_data[idx] = val;
                rgb_data[idx + 1] = val;
                rgb_data[idx + 2] = val;
            }
        }

        // Overlay red for motion pixels
        for (const ptv::PixelChange& p : detection.pixels_with_motion) {
            float abs_change = std::abs(p.change);
            float alpha = abs_change / 255.0f;
            unsigned char original = static_cast<unsigned char>(curr_img.pixels[p.y * curr_img.width + p.x]);
            int idx = (p.y * curr_img.width + p.x) * 3;
            rgb_data[idx] = static_cast<unsigned char>(original * (1 - alpha) + 255 * alpha);
            rgb_data[idx + 1] = static_cast<unsigned char>(original * (1 - alpha));
            rgb_data[idx + 2] = static_cast<unsigned char>(original * (1 - alpha));
        }

        // Overlay green for center pixels
        for (const std::pair<int, int>& center : centers) {
            int x = center.first;
            int y = center.second;
            if (x >= 0 && x < curr_img.width && y >= 0 && y < curr_img.height) {
                int idx = (y * curr_img.width + x) * 3;
                rgb_data[idx] = 0;
                rgb_data[idx + 1] = 255;
                rgb_data[idx + 2] = 0;
            }
        }

        stbi_write_png(output_path.c_str(), curr_img.width, curr_img.height, 3, rgb_data.data(), curr_img.width * 3);
    }

} // namespace ptv 