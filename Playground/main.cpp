/* main.cpp */
#include "ptv/PixelToVoxel.hpp"
#include <iostream>

int main() {
   
    
    ptv::PixelToVoxel ptv;

    std::vector<ptv::FrameInfo> frames = ptv.loadMetadata("data/metadata.json");

    for (const auto& frame : frames) {
        std::cout << "Frame " << frame.frame_index << " - Camera Index: " << frame.camera_index << " - Camera Position: " << frame.camera_position.X << ", " << frame.camera_position.Y << ", " << frame.camera_position.Z << " - Camera Rotation: " << frame.camera_rotation.X << ", " << frame.camera_rotation.Y << ", " << frame.camera_rotation.Z << " - FOV: " << frame.fov_degrees << " - Image File: " << frame.image_file << std::endl;
    }
    
    return 0;
}