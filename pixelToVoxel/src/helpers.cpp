#include "helpers.hpp"
#include <cmath>
#include <queue>

// Add stb_image_write
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

namespace ptv {

void visualize_motion(const Image& curr_img, const DetectionArray& detection, const std::string& output_path, const std::vector<std::pair<int, int>>& centers) {
    std::vector<unsigned char> rgb_data(curr_img.width * curr_img.height * 3);

    // Fill with grayscale
    for(int y = 0; y < curr_img.height; y++) {
        for(int x = 0; x < curr_img.width; x++) {
            unsigned char val = static_cast<unsigned char>(curr_img.pixels[y * curr_img.width + x]);
            int idx = (y * curr_img.width + x) * 3;
            rgb_data[idx] = val;
            rgb_data[idx + 1] = val;
            rgb_data[idx + 2] = val;
        }
    }

    // Overlay red for motion pixels
    for(const auto& p : detection.pixels_with_motion) {
        float abs_change = std::abs(p.change);
        float alpha = abs_change / 255.0f;
        unsigned char original = static_cast<unsigned char>(curr_img.pixels[p.y * curr_img.width + p.x]);
        int idx = (p.y * curr_img.width + p.x) * 3;
        rgb_data[idx] = static_cast<unsigned char>(original * (1 - alpha) + 255 * alpha); 
        rgb_data[idx + 1] = static_cast<unsigned char>(original * (1 - alpha));
        rgb_data[idx + 2] = static_cast<unsigned char>(original * (1 - alpha));
    }

    // Overlay green for center pixels
    for(const auto& center : centers) {
        int x = center.first;
        int y = center.second;
        if(x >= 0 && x < curr_img.width && y >= 0 && y < curr_img.height) {
            int idx = (y * curr_img.width + x) * 3;
            rgb_data[idx] = 0;
            rgb_data[idx + 1] = 255;
            rgb_data[idx + 2] = 0;
        }
    }

    stbi_write_png(output_path.c_str(), curr_img.width, curr_img.height, 3, rgb_data.data(), curr_img.width * 3);
}

Vec3 normalize(const Vec3& v) {
    float len = std::sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
    if (len == 0) return {0.f, 0.f, 0.f};  // Avoid division by zero
    return {v.X / len, v.Y / len, v.Z / len};
}

Vec3 apply_rotation(const Vec3& v, const Vec3& euler_deg) {
    float rx = euler_deg.X * M_PI / 180.0f;
    float ry = euler_deg.Y * M_PI / 180.0f;
    float rz = euler_deg.Z * M_PI / 180.0f;

    float cx = std::cos(rx), sx = std::sin(rx);
    float cy = std::cos(ry), sy = std::sin(ry);
    float cz = std::cos(rz), sz = std::sin(rz);

    // Rotate X
    float y1 = v.Y * cx - v.Z * sx;
    float z1 = v.Y * sx + v.Z * cx;

    // Rotate Y
    float x2 = v.X * cy + z1 * sy;
    float z2 = -v.X * sy + z1 * cy;

    // Rotate Z
    float x3 = x2 * cz - y1 * sz;
    float y3 = x2 * sz + y1 * cz;

    return {x3, y3, z2};
}

Vec3 get_ray_direction(const FrameInfo& info, int pixel_x, int pixel_y, int img_width, int img_height) {
    // Aspect ratio (assuming FOV is horizontal)
    float aspect = static_cast<float>(img_height) / img_width;
    float fov_rad = info.fov_degrees * M_PI / 180.0f;
    float tan_half_fov = std::tan(fov_rad / 2.0f);

    // Horizontal direction component: how far the ray points left/right from the center of the image
    float nx = (2.0f * (pixel_x + 0.5f) / img_width - 1.0f) * tan_half_fov;

    // Vertical direction component: how far the ray points up/down from the center of the image
    // Y-axis is inverted because image coordinates start from the top, but +Y points upward in 3D
    float ny = (1.0f - 2.0f * (pixel_y + 0.5f) / img_height) * tan_half_fov * aspect;

    // Unit vector pointing in the direction this pixel "looks" in camera space
    Vec3 local_dir = normalize({nx, ny, -1.0f});

    // Apply camera rotation to get world direction
    Vec3 world_dir = apply_rotation(local_dir, info.camera_rotation);

    return normalize(world_dir);  // Ensure normalized
}

std::vector<Vec3> find_voxel_cluster_centroids(const std::vector<int>& camera_count, int min_count, const Vec3& grid_min, float voxel_size, int N) {
    std::vector<Vec3> centroids;
    std::vector<bool> visited(static_cast<size_t>(N) * N * N, false);
    const int dirs[6][3] = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};

    auto idx = [N](int x, int y, int z) { return z * (N * N) + y * N + x; };
    auto is_valid = [N](int x, int y, int z) { return x >= 0 && x < N && y >= 0 && y < N && z >= 0 && z < N; };

    for (int z = 0; z < N; ++z) {
        for (int y = 0; y < N; ++y) {
            for (int x = 0; x < N; ++x) {
                size_t cur_idx = idx(x, y, z);
                if (camera_count[cur_idx] >= min_count && !visited[cur_idx]) {
                    // Start BFS for new cluster
                    std::queue<std::tuple<int, int, int>> q;
                    q.push({x, y, z});
                    visited[cur_idx] = true;
                    Vec3 sum = {0.f, 0.f, 0.f};
                    int count = 0;

                    while (!q.empty()) {
                        auto [cx, cy, cz] = q.front(); q.pop();
                        sum.X += grid_min.X + (static_cast<float>(cx) + 0.5f) * voxel_size;
                        sum.Y += grid_min.Y + (static_cast<float>(cy) + 0.5f) * voxel_size;
                        sum.Z += grid_min.Z + (static_cast<float>(cz) + 0.5f) * voxel_size;
                        ++count;

                        for (const auto& d : dirs) {
                            int nx = cx + d[0], ny = cy + d[1], nz = cz + d[2];
                            if (is_valid(nx, ny, nz)) {
                                size_t nidx = idx(nx, ny, nz);
                                if (camera_count[nidx] >= min_count && !visited[nidx]) {
                                    visited[nidx] = true;
                                    q.push({nx, ny, nz});
                                }
                            }
                        }
                    }

                    if (count > 0) {
                        centroids.push_back({sum.X / count, sum.Y / count, sum.Z / count});
                    }
                }
            }
        }
    }

    return centroids;
}

void visualize_flight_path(const Image& base_img, const std::vector<std::pair<int, int>>& all_centers, const std::string& output_path) {
    std::vector<unsigned char> rgb_data(base_img.width * base_img.height * 3);

    for (int y = 0; y < base_img.height; y++) {
        for (int x = 0; x < base_img.width; x++) {
            unsigned char val = static_cast<unsigned char>(base_img.pixels[y * base_img.width + x]);
            int idx = (y * base_img.width + x) * 3;
            rgb_data[idx] = val;
            rgb_data[idx + 1] = val;
            rgb_data[idx + 2] = val;
        }
    }

    // overlay green for all object centers (past path)
    for (size_t i = 0; i < all_centers.size(); ++i) {
        int x = all_centers[i].first;
        int y = all_centers[i].second;
        if (x >= 0 && x < base_img.width && y >= 0 && y < base_img.height) {
            int idx = (y * base_img.width + x) * 3;

            if (i == all_centers.size() - 1) {
                // last position in red
                rgb_data[idx] = 255;      // red
                rgb_data[idx + 1] = 0;    // green
                rgb_data[idx + 2] = 0;    // blue
            } else {
                // past positions in green
                rgb_data[idx] = 0;        // red
                rgb_data[idx + 1] = 255;  // green
                rgb_data[idx + 2] = 0;    // blue
            }
        }
    }

    stbi_write_png(output_path.c_str(), base_img.width, base_img.height, 3, rgb_data.data(), base_img.width * 3);
}

} // namespace ptv 