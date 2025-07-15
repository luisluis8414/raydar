#include "vector_ops.hpp"
#include <cmath>

namespace ptv {

    Vec3 normalize(const Vec3& v) {
        float len = std::sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
        if (len == 0) return { 0.f, 0.f, 0.f };  // Avoid division by zero
        return { v.X / len, v.Y / len, v.Z / len };
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

        return { x3, y3, z2 };
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
        Vec3 local_dir = normalize({ nx, ny, -1.0f });

        // Apply camera rotation to get world direction
        Vec3 world_dir = apply_rotation(local_dir, info.camera_rotation);

        return normalize(world_dir);  // Ensure normalized
    }

} // namespace ptv 