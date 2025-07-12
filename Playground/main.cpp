/* main.cpp */
#include "PixelToVoxel.hpp"
#include <iostream>

int main() {
    ptv::PixelToVoxel ptv;

    ptv.generateVoxelGrid("data/metadata.json");
    
    return 0;
}