#include "PixelToVoxel.hpp"
#include <iostream>

int main() {
    ptv::PixelToVoxel converter;
    
    if (converter.convertPixelToVoxel()) {
        std::cout << "Conversion successful!" << std::endl;
    } else {
        std::cout << "Conversion failed!" << std::endl;
    }
    
    return 0;
} 