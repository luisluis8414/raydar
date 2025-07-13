#include "PixelToVoxel.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <optional>

#include <nlohmann/json.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

namespace ptv {

std::map<int, std::vector<FrameInfo>> loadMetadata(const std::string& metadata_file) {
    std::map<int, std::vector<FrameInfo>> camera_frames;

    std::ifstream ifs(metadata_file);
    if (!ifs.is_open()) {
        std::cerr << "ERROR: Cannot open " << metadata_file << std::endl;
        std::exit(EXIT_FAILURE);
    }

    try {
        nlohmann::json json_data;
        ifs >> json_data;

        for (const auto& frame : json_data) {
            FrameInfo info;
            info.camera_index = frame["camera_index"];
            info.frame_index = frame["frame_index"];
            
            // Parse camera rotation
            info.camera_rotation.X = frame["camera_rotation"]["X"];
            info.camera_rotation.Y = frame["camera_rotation"]["Y"];
            info.camera_rotation.Z = frame["camera_rotation"]["Z"];
            
            info.fov_degrees = frame["fov_degrees"];
            info.image_file = frame["image_file"];
            
            // Parse camera position
            info.camera_position.X = frame["camera_position"]["X"];
            info.camera_position.Y = frame["camera_position"]["Y"];
            info.camera_position.Z = frame["camera_position"]["Z"];

            // Find the insertion point to maintain sorted order
            auto& frames = camera_frames[info.camera_index];
            auto insert_pos = std::lower_bound(frames.begin(), frames.end(), info,
                [](const FrameInfo& a, const FrameInfo& b) {
                    return a.frame_index < b.frame_index;
                });
            frames.insert(insert_pos, info);
        }
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "ERROR: JSON parsing failed: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return camera_frames;
}   

void load_image(const std::string& path, Image& out) {
    int width, height, channels;
    unsigned char* data = stbi_load(path.c_str(), &width, &height, &channels, 0);

    if(!data) {
        std::cerr << "ERROR: Failed to load image " << path << std::endl;
        std::exit(EXIT_FAILURE);
    }

    out.width = width;
    out.height = height;
    out.pixels.resize(width * height);

    for(size_t i = 0; i < width * height; i++) {
        out.pixels[i] = data[i];
    }

    stbi_image_free(data);
}

DetectionArray detectMotion(const Image& prev_img, const Image& curr_img, float motion_threshold){
    DetectionArray detection_array;
    detection_array.width = curr_img.width;
    detection_array.height = curr_img.height;

    std::vector<PixelChange> pixels_with_motion;

    for(int y = 0; y < curr_img.height; y++) {
        for(int x = 0; x < curr_img.width; x++) {
            float prev_pixel = prev_img.pixels[y * prev_img.width + x];
            float curr_pixel = curr_img.pixels[y * curr_img.width + x];

            float change = curr_pixel - prev_pixel;
            if(abs(change) > motion_threshold) {
                pixels_with_motion.push_back({x, y, change});
            }
        }
    }

    detection_array.pixels_with_motion = pixels_with_motion;
    return detection_array;
}

void generateVoxelGrid(const std::string& metadata_file_path) {
    std::map<int, std::vector<FrameInfo>> camera_frames = loadMetadata(metadata_file_path);

    std::vector<float> voxel_grid(VOXEL_GRID_N * VOXEL_GRID_N * VOXEL_GRID_N, 0.f);

    for (const auto& [camera_id, frames] : camera_frames) {
        if(frames.size() < 2) {
            std::cerr << "WARNING: Camera " << camera_id << " has less than 2 frames, skipping" << std::endl;
            continue;
        }

        std::optional<Image> prev_img;
        FrameInfo prev_info;

        for(size_t i = 0; i < frames.size(); i++) {
            FrameInfo curr_info = frames[i];
            Image curr_img; 

            load_image(curr_info.image_file, curr_img);

            if (!prev_img.has_value()) {
                prev_img = curr_img; 
                prev_info = curr_info;
                continue;
            }

            DetectionArray detection_array = detectMotion(*prev_img, curr_img, MOTION_THRESHOLD);   
            
            Vec3 camera_position = curr_info.camera_position;
            Vec3 camera_rotation = curr_info.camera_rotation;
            float fov_degrees = curr_info.fov_degrees;

            
        }
        prev_img.reset();   
    }
}

} // namespace ptv 