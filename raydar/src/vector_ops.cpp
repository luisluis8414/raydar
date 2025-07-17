#include "vector_ops.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace raydar {

    Eigen::Vector3f apply_rotation(const Eigen::Vector3f& v, const Eigen::Vector3f& euler_deg) {
        float rx_rad = euler_deg.x() * (M_PI / 180.0f);
        float ry_rad = euler_deg.y() * (M_PI / 180.0f);
        float rz_rad = euler_deg.z() * (M_PI / 180.0f);

        Eigen::AngleAxisf rotation_x(rx_rad, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rotation_y(ry_rad, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rotation_z(rz_rad, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf rotation_quat = rotation_z * rotation_y * rotation_x;

        return rotation_quat * v;
    }

    // computes a 3D direction vector (ray) that originates from the camera center
    // and passes through a specific pixel on the image sensor
    Eigen::Vector3f get_ray_direction(const FrameInfo& info, int pixel_x, int pixel_y, int img_width, int img_height) {
        // assume FOV is horizontal 
        float hFOV_rad = info.FOV_deg * M_PI / 180.0f;

        // compute tan(FOV/2), which relates the horizontal extent of the image plane
        // at unit distance (z = -1) to the FOV angle
        float tan_half_horizontal_fov = std::tan(hFOV_rad / 2.0f);

        float aspect_ratio = static_cast<float>(img_height) / img_width;

        // convert pixel coordinates to normalized device coordinates in [-1, 1],
        // so that (0, 0) maps to the center of the image
        // the offset of 0.5 centers the ray within the pixel
        float camera_x = (2.0f * (pixel_x + 0.5f) / img_width - 1.0f) * tan_half_horizontal_fov;
        float camera_y = (1.0f - 2.0f * (pixel_y + 0.5f) / img_height) * tan_half_horizontal_fov * aspect_ratio;
        float camera_z = -1.0f;
        // at this point the ray is defined in the camera local coordinate system:
        // camera is at the origin (0,0,0), looking along negative z, image plane lies at z = -1
        // this makes projection math simpler, 
        // bc we dont have to deal with arbitrary camera rotations or translations at this point

        // normalize direction in camera space
        Eigen::Vector3f local_dir(camera_x, camera_y, camera_z);
        local_dir.normalize();

        // transform the ray direction from camera space to world space
        Eigen::Vector3f world_dir = apply_rotation(local_dir, info.camera_rotation);

        world_dir.normalize();

        return world_dir;
    }

} // namespace raydar