// movement_detection.cpp
#include "movement_detection.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include "logger.hpp"
#include "sparse_voxel.hpp"
#include "vector_ops.hpp"
#include "visualization.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

namespace raydar {

std::map<int, std::vector<FrameInfo>> load_metadata(const std::string& metadata_file) {
    std::map<int, std::vector<FrameInfo>> camera_frames;

    std::ifstream ifs(metadata_file);
    if (!ifs.is_open()) {
        std::cerr << "ERROR: Cannot open " << metadata_file << std::endl;
        std::exit(EXIT_FAILURE);
    }

    try {
        nlohmann::json json_data;
        ifs >> json_data;

        for (const nlohmann::json& frame : json_data) {
            FrameInfo info;
            info.camera_index = frame["camera_index"];
            info.frame_index = frame["frame_index"];

            // Parse camera rotation
            info.camera_rotation.x() = frame["camera_rotation"]["X"];
            info.camera_rotation.y() = frame["camera_rotation"]["Y"];
            info.camera_rotation.z() = frame["camera_rotation"]["Z"];

            info.FOV_deg = frame["fov_degrees"];
            info.image_file = frame["image_file"];

            // Parse camera position
            info.camera_position.x() = frame["camera_position"]["X"];
            info.camera_position.y() = frame["camera_position"]["Y"];
            info.camera_position.z() = frame["camera_position"]["Z"];

            // Find the insertion point to maintain sorted order
            std::vector<raydar::FrameInfo>& frames = camera_frames[info.camera_index];
            std::vector<raydar::FrameInfo>::iterator insert_pos =
                std::lower_bound(frames.begin(), frames.end(), info,
                                 [](const raydar::FrameInfo& a, const raydar::FrameInfo& b) {
                                     return a.frame_index < b.frame_index;
                                 });
            frames.insert(insert_pos, info);
        }
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "ERROR: JSON parsing failed: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return camera_frames;
}

void load_image(const std::string& path, Image& out) {
    int width, height, channels;
    unsigned char* data = stbi_load(path.c_str(), &width, &height, &channels, 0);

    if (!data) {
        std::cerr << "ERROR: Failed to load image " << path << std::endl;
        std::exit(EXIT_FAILURE);
    }

    out.width = width;
    out.height = height;
    out.pixels.resize(width * height);

    for (size_t i = 0; i < width * height; i++) {
        out.pixels[i] = data[i];
    }

    stbi_image_free(data);
}

DetectionArray detect_motion(const Image& prev_img, const Image& curr_img,
                             const DetectionArray& prev_detection, float motion_threshold) {
    DetectionArray detection_array;
    detection_array.width = curr_img.width;
    detection_array.height = curr_img.height;

    std::vector<PixelChange> pixels_with_motion;

    // Create a set of previous detection coordinates for fast lookup
    std::set<PixelChange> prev_detection_set;
    for (size_t i = 0; i < prev_detection.pixels_with_motion.size(); i++) {
        const PixelChange& prev_pixel_change = prev_detection.pixels_with_motion[i];
        prev_detection_set.insert(
            PixelChange{prev_pixel_change.x, prev_pixel_change.y, prev_pixel_change.change});
    }

    for (int y = 0; y < curr_img.height; y++) {
        for (int x = 0; x < curr_img.width; x++) {
            float prev_pixel = prev_img.pixels[y * prev_img.width + x];
            float curr_pixel = curr_img.pixels[y * curr_img.width + x];

            float change = curr_pixel - prev_pixel;
            if (abs(change) > motion_threshold) {
                // Check if this pixel is already in the previous detection set
                PixelChange current_pixel{x, y, change};
                if (prev_detection_set.find(current_pixel) == prev_detection_set.end()) {
                    // Not found in previous detection, add to motion list
                    pixels_with_motion.push_back(current_pixel);
                }
            }
        }
    }

    detection_array.pixels_with_motion = pixels_with_motion;
    return detection_array;
}

// use a flood fill method to group connected pixels into objects and calculate
// their centers this reduces the total number of rays by focusing on object
// centers instead of every individual pixel
std::vector<std::pair<int, int>> find_object_centers(const DetectionArray& da) {
    // store all detected pixel movement in a set for efficiency
    std::set<std::pair<int, int>> unprocessed_pixels;
    for (const raydar::PixelChange& p : da.pixels_with_motion) {
        unprocessed_pixels.insert({p.x, p.y});
    }

    std::vector<std::pair<int, int>> centers;

    // process each object using flood fill
    while (!unprocessed_pixels.empty()) {
        // start the flood fill with any pixel from the set
        std::pair<int, int> start = *unprocessed_pixels.begin();
        std::queue<std::pair<int, int>> q;
        q.push(start);
        unprocessed_pixels.erase(start);

        // track the sum of coordinates and the number of pixels in the current
        // object
        double sum_x = 0.0;
        double sum_y = 0.0;
        int count = 0;

        // flood fill to find all connected pixels
        while (!q.empty()) {
            std::pair<int, int> xy = q.front();
            int x = xy.first;
            int y = xy.second;
            q.pop();

            sum_x += x;
            sum_y += y;
            ++count;

            // check all neighbours of the current pixel (3x3)
            for (int dx = -2; dx <= 2; ++dx) {
                for (int dy = -2; dy <= 2; ++dy) {
                    if (dx == 0 && dy == 0)
                        continue;
                    std::pair<int, int> neigh = {x + dx, y + dy};

                    // if this neighbour is in the set of unprocessed pixels, add it to
                    // the object
                    if (unprocessed_pixels.count(neigh)) {  // no bound checks needed, all pixels in
                        // unprocessed_pixels are valid
                        unprocessed_pixels.erase(neigh);
                        // check its neighbours to map out the entire object
                        q.push(neigh);
                    }
                }
            }
        }

        // calc the center of the found object
        int center_x = static_cast<int>(std::round(sum_x / count));
        int center_y = static_cast<int>(std::round(sum_y / count));
        centers.push_back({center_x, center_y});
    }

    return centers;
}

// frame means a snapshot where all cameras see the same scene (from diffrent angles/positions)
std::vector<RaysAtFrame> group_rays_by_frame(
    const std::map<int, std::map<int, std::vector<Eigen::Vector3f>>>& rays,
    const std::map<int, std::map<int, Eigen::Vector3f>>& camera_positions) {
    std::map<int, std::vector<Ray>> temp_frame_rays;

    // iterate over all cameras
    for (const auto& cam_it : rays) {
        int camera_id = cam_it.first;
        const auto& frame_to_rays = cam_it.second;

        // iterate over all frames for the current camera
        for (const auto& frame_it : frame_to_rays) {
            int frame = frame_it.first;
            const auto& ray_list = frame_it.second;

            // find the camera position for the current frame
            auto pos_it = camera_positions.find(camera_id);
            if (pos_it != camera_positions.end()) {
                auto frame_pos_it = pos_it->second.find(frame);
                if (frame_pos_it != pos_it->second.end()) {
                    Eigen::Vector3f camera_pos = frame_pos_it->second;

                    for (const auto& ray_dir : ray_list) {
                        temp_frame_rays[frame].push_back({camera_id, ray_dir, camera_pos});
                    }
                }
            }
        }
    }

    std::vector<RaysAtFrame> frame_rays;
    for (const auto& entry : temp_frame_rays) {
        frame_rays.push_back({entry.first, entry.second});
    }

    // sort by frame
    std::sort(frame_rays.begin(), frame_rays.end(),
              [](const RaysAtFrame& a, const RaysAtFrame& b) { return a.frame_id < b.frame_id; });

    return frame_rays;
}

void generate_flight_path_images(
    const std::map<int, std::vector<FrameInfo>>& camera_frames,
    const std::map<int, std::vector<std::pair<int, int>>>& all_object_centers) {
    for (std::map<int, std::vector<FrameInfo>>::const_iterator cam = camera_frames.begin();
         cam != camera_frames.end(); ++cam) {
        int camera_id = cam->first;
        const std::vector<FrameInfo>& frames = cam->second;
        if (frames.empty())
            continue;

        // use the last frame as the base image
        Image base_img;
        load_image(frames.back().image_file, base_img);

        const std::vector<std::pair<int, int>>& all_centers = all_object_centers.at(camera_id);

        std::string flight_path_dir = "motion_output/flight_paths";
        std::filesystem::create_directories(flight_path_dir);

        std::string output_name = "motion_camera" + std::to_string(camera_id) + "_flight_path.png";
        std::string output_path = flight_path_dir + "/" + output_name;

        visualize_flight_path(base_img, all_centers, output_path);
    }
}
void write_3d_points_to_file(const std::map<int, std::vector<Eigen::Vector3f>>& frame_points,
                             const std::string& output_file) {
    std::ofstream out(output_file);
    if (!out.is_open()) {
        std::cerr << "ERROR: Cannot open " << output_file << std::endl;
        return;
    }
    for (std::map<int, std::vector<Eigen::Vector3f>>::const_iterator it = frame_points.begin();
         it != frame_points.end(); ++it) {
        int frame = it->first;
        const std::vector<Eigen::Vector3f>& points = it->second;
        for (size_t i = 0; i < points.size(); ++i) {
            const Eigen::Vector3f& pt = points[i];
            out << frame << ": " << std::fixed << std::setprecision(3) << pt.x() << " "
                << std::fixed << std::setprecision(3) << pt.y() << " " << std::fixed
                << std::setprecision(3) << pt.z() << std::endl;
        }
    }
}

void detect_objects(const std::string& metadata_file_path, float detect_motion_threshold,
                    float min_distance) {
    if (!logger::initialize()) {
        std::cerr << "WARN: Logging system failed to initialize\n";
        return;
    }

    std::map<int, std::vector<FrameInfo>> camera_frames = load_metadata(metadata_file_path);
    std::map<int, std::map<int, std::vector<Eigen::Vector3f>>> rays;
    std::map<int, std::map<int, Eigen::Vector3f>> camera_positions;
    std::map<int, std::vector<std::pair<int, int>>> all_object_centers;  // for flight path

    GridExtent extent = compute_grid_extent(camera_frames);
    float max_distance = 2000.0f;

    for (std::map<int, std::vector<raydar::FrameInfo>>::const_iterator camera_frames_pair =
             camera_frames.begin();
         camera_frames_pair != camera_frames.end(); ++camera_frames_pair) {
        int camera_id = camera_frames_pair->first;
        const std::vector<raydar::FrameInfo>& frames = camera_frames_pair->second;

        if (frames.size() < 2) {
            std::cerr << "WARNING: Camera " << camera_id << " has less than 2 frames, skipping"
                      << std::endl;
            continue;
        }

        std::optional<Image> prev_img;
        FrameInfo prev_info;
        DetectionArray prev_detection;

        for (size_t i = 0; i < frames.size(); i++) {
            FrameInfo curr_info = frames[i];
            Image curr_img;
            load_image(curr_info.image_file, curr_img);

            if (!prev_img.has_value()) {
                prev_img = curr_img;
                prev_info = curr_info;
                continue;
            }

            DetectionArray detection_array =
                detect_motion(*prev_img, curr_img, prev_detection, detect_motion_threshold);
            prev_detection = detection_array;

            std::vector<std::pair<int, int>> object_centers = find_object_centers(detection_array);

            all_object_centers[camera_id].insert(all_object_centers[camera_id].end(),
                                                 object_centers.begin(), object_centers.end());

            {
                std::string output_dir = "motion_output";
                std::string output_name = "motion_camera" + std::to_string(curr_info.camera_index) +
                                          "_frame" + std::to_string(curr_info.frame_index) + ".png";
                std::string output_path = output_dir + "/" + output_name;
                std::filesystem::create_directories(output_dir);
                visualize_motion(curr_img, detection_array, output_path, object_centers);
            }

            for (const std::pair<int, int>& center : object_centers) {
                int x = center.first;
                int y = center.second;
                Eigen::Vector3f dir =
                    get_ray_direction(curr_info, x, y, curr_img.width, curr_img.height);
                rays[camera_id][curr_info.frame_index].push_back(dir);
                camera_positions[camera_id][curr_info.frame_index] = curr_info.camera_position;

                logger::log_ray_directions(camera_id, curr_info.frame_index, x, y,
                                           curr_info.camera_position, dir);
                logger::log_formatted_ray(curr_info.camera_position, dir, 2000, camera_id,
                                          curr_info.frame_index);
            }

            prev_img = curr_img;
            prev_info = curr_info;
        }
    }

    generate_flight_path_images(camera_frames, all_object_centers);

    // group rays by frame for per-frame processing
    std::vector<RaysAtFrame> frame_rays = group_rays_by_frame(rays, camera_positions);

    // store detected object coordinates per frame
    std::map<int, std::vector<Eigen::Vector3f>> frame_points;

    // threshold for density e.g. at least 2 rays intersecting in a voxel
    float density_threshold = 2.0f;

    for (const auto& frame : frame_rays) {
        SparseVoxelGrid grid;
        for (const auto& ray : frame.rays) {
            // apply start offset = 2 * voxel_size to skip camera-near voxels
            float start_offset = 20.0f * extent.voxel_size;
            fill_sparse_voxel_grid(grid, ray.camera_position, ray.ray, 1.0f, max_distance, extent,
                                   start_offset);
        }
        std::vector<Eigen::Vector3f> points =
            extract_high_density_points(grid, extent, density_threshold);
        frame_points[frame.frame_id] = points;
    }

    write_3d_points_to_file(frame_points, "detected_objects.txt");

    logger::shutdown();
}
}  // namespace raydar