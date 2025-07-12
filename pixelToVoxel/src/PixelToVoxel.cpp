#include "PixelToVoxel.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>

namespace ptv {

PixelToVoxel::PixelToVoxel() {
}

PixelToVoxel::~PixelToVoxel() {
}

std::map<int, std::vector<FrameInfo>> PixelToVoxel::loadMetadata(const std::string& metadata_file) {
    std::map<int, std::vector<FrameInfo>> camera_frames;

    std::ifstream ifs(metadata_file);
    if (!ifs.is_open()) {
        std::cerr << "ERROR: Cannot open " << metadata_file << std::endl;
        return camera_frames;
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
        camera_frames.clear();
    }

    return camera_frames;
}   

bool PixelToVoxel::convertPixelToVoxel() {
    // Implementation will go here
    return true;
}

} // namespace ptv 