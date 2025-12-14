#pragma once
#include <Eigen/Dense>
#include <string>

namespace logger {

/**
 * @brief Initializes the logging system by creating the log file and directory
 *
 * @param log_dir Directory where the log file will be stored (default: "logs")
 * @param log_file Name of the log file (default: "coordinates.log")
 * @return true if the log file was successfully created, false otherwise
 */
bool initialize(const std::string& log_dir = "logs", const std::string& log_file = "rays.log");

/**
 * @brief Logs the camera position and ray direction for a detected object
 *
 * @param camera_id ID of the camera
 * @param frame_index Index of the frame
 * @param pixel_x X-coordinate of the pixel in the image
 * @param pixel_y Y-coordinate of the pixel in the image
 * @param camera_pos Camera position in world coordinates
 * @param ray_dir Ray direction in world coordinates
 */
void log_ray_directions(int camera_id, int frame_index, int pixel_x, int pixel_y,
                        const Eigen::Vector3f& camera_pos, const Eigen::Vector3f& ray_dir);

/**
 * @brief Logs the scaled ray coordinates in GeoGebra format.
 *
 * @param camera_pos Camera position in world coordinates.
 * @param unit_vector Unit vector representing the ray direction.
 * @param scale_factor Scaling factor for the ray.
 */
void log_formatted_ray(const Eigen::Vector3f& camera_pos, const Eigen::Vector3f& unit_vector,
                       float scale_factor, int camera_id, int frame_index);

void shutdown();

}  // namespace logger