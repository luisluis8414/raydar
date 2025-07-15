/* main.cpp */
#include "PixelToVoxel.hpp"

int main() {

    ptv::generate_voxel_grid("data/metadata.json", 2.0f);

    return 0;
}