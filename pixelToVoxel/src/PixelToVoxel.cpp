#include "ptv/PixelToVoxel.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

namespace ptv {

PixelToVoxel::PixelToVoxel() {
}

PixelToVoxel::~PixelToVoxel() {
}

std::vector<FrameInfo> PixelToVoxel::loadMetadata(const std::string& metadata_file) {
    std::vector<FrameInfo> frames;

    std::ifstream ifs(metadata_file);
    if (!ifs.is_open()) {
        std::cerr << "ERROR: Cannot open " << metadata_file << std::endl;
        return frames;
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

            frames.push_back(info);
        }
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "ERROR: JSON parsing failed: " << e.what() << std::endl;
        frames.clear();
    }

    return frames;
}   

bool PixelToVoxel::convertPixelToVoxel() {
    // Implementation will go here
    return true;
}

} // namespace ptv 