#pragma once

#include <Eigen/Dense>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "movement_detection.hpp"

// custom hash for voxel key to enable unordered_map usage
struct VoxelKeyHash {
    std::size_t operator()(const std::tuple<int, int, int>& key) const;
};

using SparseVoxelGrid = std::unordered_map<std::tuple<int, int, int>, float, VoxelKeyHash>;

// defines grid bounds and resolution
struct GridExtent {
    Eigen::Vector3f min;  // minimum bounds of the grid
    Eigen::Vector3f max;  // maximum bounds of the grid
    float voxel_size;     // size of each voxel
};

GridExtent compute_grid_extent(const std::map<int, std::vector<raydar::FrameInfo>>& camera_frames,
                               float buffer = 1000.0f);

bool ray_aabb_intersection(const Eigen::Vector3f& ray_origin, const Eigen::Vector3f& ray_dir,
                           const Eigen::Vector3f& box_min, const Eigen::Vector3f& box_max,
                           float& t_entry, float& t_exit);

void fill_sparse_voxel_grid(SparseVoxelGrid& grid, const Eigen::Vector3f& ray_origin,
                            const Eigen::Vector3f& ray_dir, float value, float max_distance,
                            const GridExtent& extent, float start_offset = 0.0f);

void save_sparse_voxel_grid(const SparseVoxelGrid& grid, const std::string& output_file,
                            const GridExtent& extent);

// extracts world coordinates of voxels with density >= threshold
std::vector<Eigen::Vector3f> extract_high_density_points(const SparseVoxelGrid& grid,
                                                         const GridExtent& extent, float threshold);