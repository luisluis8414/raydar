/* main.cpp */
#include "PixelToVoxel.hpp"
#include <iostream>

int main() {
    std::cout << "Testing PixelToVoxel library..." << std::endl;
    
    ptv::PixelToVoxel converter;
    if (converter.convertPixelToVoxel()) {
        std::cout << "Pixel to Voxel conversion successful!" << std::endl;
    } else {
        std::cout << "Pixel to Voxel conversion failed!" << std::endl;
    }
    
    return 0;
}