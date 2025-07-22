#include "sparse_voxel.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

#include "movement_detection.hpp"

// custom hash for voxel key to enable unordered_map usage
std::size_t VoxelKeyHash::operator()(const std::tuple<int, int, int>& key) const {
    auto [x, y, z] = key;
    return std::hash<int>()(x) ^ (std::hash<int>()(y) << 1) ^ (std::hash<int>()(z) << 2);
}

// computes grid extent dynamically from camera frames with buffer
GridExtent compute_grid_extent(const std::map<int, std::vector<raydar::FrameInfo>>& camera_frames,
                               float buffer) {
    Eigen::Vector3f min_pos = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f max_pos = Eigen::Vector3f::Constant(std::numeric_limits<float>::min());

    for (const auto& cam : camera_frames) {
        for (const auto& frame : cam.second) {
            min_pos = min_pos.cwiseMin(frame.camera_position);
            max_pos = max_pos.cwiseMax(frame.camera_position);
        }
    }

    // add buffer to extents
    min_pos -= Eigen::Vector3f(buffer, buffer, buffer);
    max_pos += Eigen::Vector3f(buffer, buffer, buffer);

    return {min_pos, max_pos, 1.0f};  // voxel_size = x meters
}

// computes ray intersection with axis aligned bounding box
bool ray_aabb_intersection(const Eigen::Vector3f& ray_origin, const Eigen::Vector3f& ray_dir,
                           const Eigen::Vector3f& box_min, const Eigen::Vector3f& box_max,
                           float& t_entry, float& t_exit) {
    t_entry = 0.0f;
    t_exit = std::numeric_limits<float>::infinity();

    for (int i = 0; i < 3; ++i) {
        float inv_dir = 1.0f / ray_dir[i];
        float t1 = (box_min[i] - ray_origin[i]) * inv_dir;
        float t2 = (box_max[i] - ray_origin[i]) * inv_dir;

        if (std::isnan(t1) || std::isnan(t2)) {
            if (ray_origin[i] < box_min[i] || ray_origin[i] > box_max[i]) {
                return false;
            }
            continue;
        }

        if (t1 > t2)
            std::swap(t1, t2);
        t_entry = std::max(t_entry, t1);
        t_exit = std::min(t_exit, t2);

        if (t_entry > t_exit)
            return false;
    }

    return true;
}

// fills sparse voxel grid along ray path using dda traversal
void fill_sparse_voxel_grid(SparseVoxelGrid& grid, const Eigen::Vector3f& ray_origin,
                            const Eigen::Vector3f& ray_dir, float value, float max_distance,
                            const GridExtent& extent, float start_offset) {
    float t_entry, t_exit;
    if (!ray_aabb_intersection(ray_origin, ray_dir, extent.min, extent.max, t_entry, t_exit)) {
        return;
    }
    t_entry = std::max(t_entry, 0.0f);
    // apply start offset to skip initial voxels near camera
    t_entry = std::max(t_entry, start_offset);
    t_exit = std::min(t_exit, max_distance);
    if (t_entry >= t_exit)
        return;

    Eigen::Vector3f start_pos = ray_origin + t_entry * ray_dir;

    // AW init
    Eigen::Vector3i current_idx;
    for (int i = 0; i < 3; ++i) {
        current_idx[i] =
            static_cast<int>(std::floor((start_pos[i] - extent.min[i]) / extent.voxel_size));
    }

    Eigen::Vector3i step = ray_dir.cwiseSign().cast<int>();
    Eigen::Vector3f t_delta = (extent.voxel_size / ray_dir.cwiseAbs().array()).matrix();
    Eigen::Vector3f t_max = Eigen::Vector3f::Zero();
    for (int i = 0; i < 3; ++i) {
        if (step[i] != 0) {
            float next_boundary =
                extent.min[i] + (current_idx[i] + (step[i] > 0 ? 1 : 0)) * extent.voxel_size;
            t_max[i] = (next_boundary - ray_origin[i]) / ray_dir[i];
        } else {
            t_max[i] = std::numeric_limits<float>::infinity();  // Handle parallel rays
        }
    }

    float t_current = t_entry;
    while (t_current < t_exit) {
        auto key = std::make_tuple(current_idx[0], current_idx[1], current_idx[2]);
        grid[key] += value;

        int axis = 0;
        if (t_max[1] < t_max[axis])
            axis = 1;
        if (t_max[2] < t_max[axis])
            axis = 2;

        t_current = std::min(t_max[axis], t_exit);
        current_idx[axis] += step[axis];
        t_max[axis] += t_delta[axis];
    }
}

void save_sparse_voxel_grid(const SparseVoxelGrid& grid, const std::string& output_file,
                            const GridExtent& extent) {
    std::ofstream ofs(output_file, std::ios::binary);
    if (!ofs) {
        std::cerr << "ERROR: Cannot open " << output_file << " for writing\n";
        return;
    }

    size_t num_voxels = grid.size();
    ofs.write(reinterpret_cast<const char*>(&num_voxels), sizeof(size_t));

    ofs.write(reinterpret_cast<const char*>(&extent.min[0]), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&extent.min[1]), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&extent.min[2]), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&extent.voxel_size), sizeof(float));

    for (const auto& entry : grid) {
        auto [x, y, z] = entry.first;
        float val = entry.second;
        ofs.write(reinterpret_cast<const char*>(&x), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(&y), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(&z), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(&val), sizeof(float));
    }
}

// extracts world coordinates of voxels with density >= threshold
std::vector<Eigen::Vector3f> extract_high_density_points(const SparseVoxelGrid& grid,
                                                         const GridExtent& extent,
                                                         float threshold) {
    std::vector<Eigen::Vector3f> points;
    for (const auto& entry : grid) {
        auto [ix, iy, iz] = entry.first;
        float val = entry.second;
        if (val >= threshold) {
            float x = extent.min.x() + (ix + 0.5f) * extent.voxel_size;
            float y = extent.min.y() + (iy + 0.5f) * extent.voxel_size;
            float z = extent.min.z() + (iz + 0.5f) * extent.voxel_size;
            points.push_back({x, y, z});
        }
    }
    return points;
}