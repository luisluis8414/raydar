#include "vector_ops.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ptv {

    Eigen::Vector3f apply_rotation(const Eigen::Vector3f& v, const Eigen::Vector3f& euler_deg) {
        float rx_rad = euler_deg.x() * (M_PI / 180.0f);
        float ry_rad = euler_deg.y() * (M_PI / 180.0f);
        float rz_rad = euler_deg.z() * (M_PI / 180.0f);

        Eigen::AngleAxisf rotation_x(rx_rad, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rotation_y(ry_rad, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rotation_z(rz_rad, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf rotation_quat = rotation_x * rotation_y * rotation_z;

        return rotation_quat * v;
    }

    Eigen::Vector3f get_ray_direction(const FrameInfo& info, int pixel_x, int pixel_y, int img_width, int img_height) {
        // Assume FOV is horizontal to match the old implementation
        float horizontal_fov_rad = info.fov_degrees * M_PI / 180.0f;
        float tan_half_horizontal_fov = std::tan(horizontal_fov_rad / 2.0f);

        float aspect_ratio = static_cast<float>(img_height) / img_width;

        float camera_x = (2.0f * (pixel_x + 0.5f) / img_width - 1.0f) * tan_half_horizontal_fov;
        float camera_y = (1.0f - 2.0f * (pixel_y + 0.5f) / img_height) * tan_half_horizontal_fov * aspect_ratio;
        float camera_z = -1.0f;

        // Unit vector pointing in the direction this pixel looks in camera space
        Eigen::Vector3f local_dir(camera_x, camera_y, camera_z);
        local_dir.normalize();

        // Apply camera rotation to get world direction
        Eigen::Vector3f world_dir = apply_rotation(local_dir, info.camera_rotation);

        world_dir.normalize();

        return world_dir;
    }

} // namespace ptv