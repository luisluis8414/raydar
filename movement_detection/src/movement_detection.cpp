#include "movement_detection.hpp"
#include "vector_ops.hpp"
#include "visualization.hpp"

#include <fstream>
#include <iostream>
#include <algorithm>
#include <optional>
#include <set>
#include <filesystem>
#include <vector>
#include <queue>
#include <cmath>
#include <utility> 

#include <nlohmann/json.hpp>
#include <Eigen/Dense>

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

namespace ptv {

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

                info.fov_degrees = frame["fov_degrees"];
                info.image_file = frame["image_file"];

                // Parse camera position
                info.camera_position.x() = frame["camera_position"]["X"];
                info.camera_position.y() = frame["camera_position"]["Y"];
                info.camera_position.z() = frame["camera_position"]["Z"];

                // Find the insertion point to maintain sorted order
                std::vector<ptv::FrameInfo>& frames = camera_frames[info.camera_index];
                std::vector<ptv::FrameInfo>::iterator insert_pos = std::lower_bound(frames.begin(), frames.end(), info,
                    [](const ptv::FrameInfo& a, const ptv::FrameInfo& b) {
                        return a.frame_index < b.frame_index;
                    });
                frames.insert(insert_pos, info);
            }
        }
        catch (const nlohmann::json::exception& e) {
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

    DetectionArray detect_motion(const Image& prev_img, const Image& curr_img, float motion_threshold) {
        DetectionArray detection_array;
        detection_array.width = curr_img.width;
        detection_array.height = curr_img.height;

        std::vector<PixelChange> pixels_with_motion;

        for (int y = 0; y < curr_img.height; y++) {
            for (int x = 0; x < curr_img.width; x++) {
                float prev_pixel = prev_img.pixels[y * prev_img.width + x];
                float curr_pixel = curr_img.pixels[y * curr_img.width + x];

                float change = curr_pixel - prev_pixel;
                if (abs(change) > motion_threshold) {
                    pixels_with_motion.push_back({ x, y, change });
                }
            }
        }

        detection_array.pixels_with_motion = pixels_with_motion;
        return detection_array;
    }

    // use a flood fill method to group connected pixels into objects and calculate their centers
    // this reduces the total number of rays by focusing on object centers instead of every individual pixel
    std::vector<std::pair<int, int>> find_object_centers(const DetectionArray& da) {

        // store all detected pixel movement in a set for efficiency
        std::set<std::pair<int, int>> unprocessed_pixels;
        for (const ptv::PixelChange& p : da.pixels_with_motion) {
            unprocessed_pixels.insert({ p.x, p.y });
        }

        std::vector<std::pair<int, int>> centers;

        // process each object using flood fill
        while (!unprocessed_pixels.empty()) {
            // start the flood fill with any pixel from the set
            std::pair<int, int> start = *unprocessed_pixels.begin();
            std::queue<std::pair<int, int>> q;
            q.push(start);
            unprocessed_pixels.erase(start);

            // track the sum of coordinates and the number of pixels in the current object
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
                        if (dx == 0 && dy == 0) continue;
                        std::pair<int, int> neigh = { x + dx, y + dy };

                        // if this neighbour is in the set of unprocessed pixels, add it to the object
                        if (unprocessed_pixels.count(neigh)) { // no bound checks needed, all pixels in unprocessed_pixels are valid
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
            centers.push_back({ center_x, center_y });
        }

        return centers;
    }

    void generate_flight_path_images(const std::map<int, std::vector<FrameInfo>>& camera_frames,const std::map<int, std::vector<std::pair<int, int>>>& all_object_centers) {
        for (const auto& [camera_id, frames] : camera_frames) {
            if (frames.empty()) continue;

            // use the last frame as the base image
            Image base_img;
            load_image(frames.back().image_file, base_img);

            const auto& all_centers = all_object_centers.at(camera_id);

            std::string flight_path_dir = "motion_output/flight_paths";
            std::filesystem::create_directories(flight_path_dir);

            std::string output_name = "motion_camera" + std::to_string(camera_id) + "_flight_path.png";
            std::string output_path = flight_path_dir + "/" + output_name;

            visualize_flight_path(base_img, all_centers, output_path);
        }
    }

    void generate_voxel_grid(const std::string& metadata_file_path, const float detect_motion_threshold) {
        std::map<int, std::vector<FrameInfo>> camera_frames = load_metadata(metadata_file_path);

        std::map<int, std::map<int, std::vector<Eigen::Vector3f>>> rays;
        std::map<int, std::map<int, Eigen::Vector3f>> camera_positions;

        std::map<int, std::vector<std::pair<int, int>>> all_object_centers; // for flight path

        for (const std::pair<int, std::vector<ptv::FrameInfo>>& camera_frames_pair : camera_frames) {
            int camera_id = camera_frames_pair.first;
            const std::vector<ptv::FrameInfo>& frames = camera_frames_pair.second;

            if (frames.size() < 2) {
                std::cerr << "WARNING: Camera " << camera_id << " has less than 2 frames, skipping" << std::endl;
                continue;
            }

            std::optional<Image> prev_img;
            FrameInfo prev_info;

            for (size_t i = 0; i < frames.size(); i++) {
                FrameInfo curr_info = frames[i];
                Image curr_img;

                load_image(curr_info.image_file, curr_img);

                if (!prev_img.has_value()) {
                    prev_img = curr_img;
                    prev_info = curr_info;
                    continue;
                }

                DetectionArray detection_array = detect_motion(*prev_img, curr_img, detect_motion_threshold);

                std::vector<std::pair<int, int>> object_centers = find_object_centers(detection_array);

                all_object_centers[camera_id].insert(
                    all_object_centers[camera_id].end(),
                    object_centers.begin(),
                    object_centers.end()
                );

                {
                    std::string output_dir = "motion_output";
                    std::string output_name = "motion_camera" + std::to_string(curr_info.camera_index) + "_frame" + std::to_string(curr_info.frame_index) + ".png";
                    std::string output_path = output_dir + "/" + output_name;

                    std::filesystem::create_directories(output_dir);
                    visualize_motion(curr_img, detection_array, output_path, object_centers);
                }

                for (const std::pair<int, int>& center : object_centers) {
                    int x = center.first;
                    int y = center.second;
                    Eigen::Vector3f dir = get_ray_direction(curr_info, x, y, curr_img.width, curr_img.height);
                    rays[camera_id][curr_info.frame_index].push_back(dir);
                    camera_positions[camera_id][curr_info.frame_index] = curr_info.camera_position; // Already Eigen::Vector3f
                }

                prev_img = curr_img;
                prev_info = curr_info;
            }
            prev_img.reset();
        }
        generate_flight_path_images(camera_frames, all_object_centers);
    }
} // namespace ptv