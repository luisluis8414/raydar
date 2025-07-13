#include "helpers.hpp"
#include <cmath>
#include <limits>

// Add stb_image_write
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

namespace ptv {

void visualizeMotion(const Image& curr_img, const DetectionArray& detection, const std::string& output_path) {
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

    stbi_write_png(output_path.c_str(), curr_img.width, curr_img.height, 3, rgb_data.data(), curr_img.width * 3);
}

Vec3 normalize(const Vec3& v) {
    float len = std::sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
    if (len == 0) return {0.f, 0.f, 0.f};  // Avoid division by zero
    return {v.X / len, v.Y / len, v.Z / len};
}

Vec3 applyRotation(const Vec3& v, const Vec3& euler_deg) {
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

Vec3 getRayDirection(const FrameInfo& info, int pixel_x, int pixel_y, int img_width, int img_height) {
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
    Vec3 world_dir = applyRotation(local_dir, info.camera_rotation);

    return normalize(world_dir);  // Ensure normalized
}

void traceRayThroughVoxels(std::vector<float>& voxel_grid, const Vec3& origin, const Vec3& dir, float increment_value) {
    const int N = VOXEL_GRID_N;
    const float GRID_SPAN = N * VOXEL_SIZE;
    const float HALF_SPAN = GRID_SPAN / 2.0f;
    Vec3 grid_min = {GRID_CENTER.X - HALF_SPAN, GRID_CENTER.Y - HALF_SPAN, GRID_CENTER.Z - HALF_SPAN};

    // Voxel coordinates of origin
    int voxel_x = static_cast<int>((origin.X - grid_min.X) / VOXEL_SIZE);
    int voxel_y = static_cast<int>((origin.Y - grid_min.Y) / VOXEL_SIZE);
    int voxel_z = static_cast<int>((origin.Z - grid_min.Z) / VOXEL_SIZE);

    // Step directions (1 or -1)
    int step_x = (dir.X >= 0) ? 1 : -1;
    int step_y = (dir.Y >= 0) ? 1 : -1;
    int step_z = (dir.Z >= 0) ? 1 : -1;

    // Delta t (distance to next voxel boundary along each axis)
    float delta_tx = (dir.X != 0) ? VOXEL_SIZE / std::abs(dir.X) : std::numeric_limits<float>::max();
    float delta_ty = (dir.Y != 0) ? VOXEL_SIZE / std::abs(dir.Y) : std::numeric_limits<float>::max();
    float delta_tz = (dir.Z != 0) ? VOXEL_SIZE / std::abs(dir.Z) : std::numeric_limits<float>::max();

    // Max t (initial distance to first boundary)
    float frac_x = (origin.X - grid_min.X) / VOXEL_SIZE - voxel_x;
    float frac_y = (origin.Y - grid_min.Y) / VOXEL_SIZE - voxel_y;
    float frac_z = (origin.Z - grid_min.Z) / VOXEL_SIZE - voxel_z;

    float max_tx = (dir.X > 0 ? (1.0f - frac_x) : frac_x) * delta_tx;
    float max_ty = (dir.Y > 0 ? (1.0f - frac_y) : frac_y) * delta_ty;
    float max_tz = (dir.Z > 0 ? (1.0f - frac_z) : frac_z) * delta_tz;

    // Traverse voxels
    const int MAX_STEPS = 3 * N;  // Safety limit to prevent infinite loops
    int steps = 0;
    while (steps++ < MAX_STEPS) {
        // If inside grid, increment voxel
        if (voxel_x >= 0 && voxel_x < N && voxel_y >= 0 && voxel_y < N && voxel_z >= 0 && voxel_z < N) {
            int index = voxel_z * (N * N) + voxel_y * N + voxel_x;
            voxel_grid[index] += increment_value;
        } else {
            // Exit if out of bounds
            break;
        }

        // Advance to next voxel
        if (max_tx <= max_ty && max_tx <= max_tz) {
            voxel_x += step_x;
            max_tx += delta_tx;
        } else if (max_ty <= max_tz) {
            voxel_y += step_y;
            max_ty += delta_ty;
        } else {
            voxel_z += step_z;
            max_tz += delta_tz;
        }
    }
}

} // namespace ptv 