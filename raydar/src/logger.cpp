#include "logger.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace logger {

static std::ofstream g_log_file;
static std::ofstream g_formatted_log_file;
static int g_last_camera_id = -1;
static int g_last_formatted_camera_id = -1;

bool initialize(const std::string& log_dir, const std::string& log_file) {
    std::filesystem::create_directories(log_dir);

    g_log_file.open(log_dir + "/" + log_file, std::ios::out | std::ios::trunc);
    if (!g_log_file) {
        std::cerr << "ERROR: Cannot open '" << log_dir + "/" + log_file << "' for writing.\n";
        return false;
    }

    g_formatted_log_file.open(log_dir + "/rays_GeoGebra.log", std::ios::out | std::ios::trunc);
    if (!g_formatted_log_file) {
        std::cerr << "ERROR: Cannot open '" << log_dir + "/formatted_rays.log' for writing.\n";
        return false;
    }

    return true;
}

void log_ray_directions(int camera_id, int frame_index, int pixel_x, int pixel_y,
                        const Eigen::Vector3f& camera_pos, const Eigen::Vector3f& ray_dir) {
    if (!g_log_file)
        return;

    if (camera_id != g_last_camera_id) {
        g_log_file << "-----------------------\n";
        g_log_file << "Cam: " << camera_id << ", Position: [" << camera_pos.x() << ", "
                   << camera_pos.y() << ", " << camera_pos.z() << "]\n";
        g_last_camera_id = camera_id;
    }

    g_log_file << "     Frame: " << std::setw(4) << std::right << frame_index << ", Object at: ("
               << std::setw(4) << std::right << pixel_x << ", " << std::setw(4) << std::right
               << pixel_y << ")" << ", Ray Direction: [" << std::setw(10) << std::right
               << std::setprecision(6) << ray_dir.x() << " " << std::setw(10) << std::right
               << std::setprecision(6) << ray_dir.y() << " " << std::setw(10) << std::right
               << std::setprecision(6) << ray_dir.z() << "]" << '\n';
}

void log_formatted_ray(const Eigen::Vector3f& camera_pos, const Eigen::Vector3f& unit_vector,
                       float scale_factor, int camera_id, int frame_index) {
    if (!g_formatted_log_file)
        return;

    if (camera_id != g_last_formatted_camera_id) {
        g_formatted_log_file << "-----------------------\n";
        g_formatted_log_file << "Cam: " << camera_id << ", Position: [" << std::fixed
                             << std::setprecision(6) << camera_pos.x() << ", " << std::fixed
                             << std::setprecision(6) << camera_pos.y() << ", " << std::fixed
                             << std::setprecision(6) << camera_pos.z() << "]\n";
        g_last_formatted_camera_id = camera_id;
    }

    Eigen::Vector3f scaled_vector = unit_vector * scale_factor;
    Eigen::Vector3f endpoint = camera_pos + scaled_vector;

    g_formatted_log_file << "     Frame: " << std::setw(4) << std::right << frame_index
                         << ", Vector((" << std::fixed << std::setprecision(6) << camera_pos.x()
                         << ", " << std::fixed << std::setprecision(6) << camera_pos.y() << ", "
                         << std::fixed << std::setprecision(6) << camera_pos.z() << "), ("
                         << std::fixed << std::setprecision(3) << endpoint.x() << ", " << std::fixed
                         << std::setprecision(3) << endpoint.y() << ", " << std::fixed
                         << std::setprecision(3) << endpoint.z() << "))\n";
}

void shutdown() {
    if (g_log_file.is_open())
        g_log_file.close();
    if (g_formatted_log_file.is_open())
        g_formatted_log_file.close();
    g_last_camera_id = -1;
    g_last_formatted_camera_id = -1;
}

}  // namespace logger