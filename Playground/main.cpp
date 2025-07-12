/* main.cpp */
#include "PixelToVoxel.hpp"
#include <iostream>

int main() {
    ptv::PixelToVoxel ptv;

    // Load metadata
    auto camera_frames = ptv.loadMetadata("data/metadata.json");
    
    // Print some information about the loaded data
    std::cout << "Loaded metadata for " << camera_frames.size() << " cameras\n";
    
    for (const auto& [camera_id, frames] : camera_frames) {
        std::cout << "Camera " << camera_id << " has " << frames.size() << " frames\n";
    }
    
    // Call the conversion function
    ptv.convertPixelToVoxel();
    
    return 0;
}