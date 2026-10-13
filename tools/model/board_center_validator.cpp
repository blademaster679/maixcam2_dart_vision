// Exercise the exact C++ application validator without requiring a camera.
#include "dart/green_detector.hpp"
#include "maix_image.hpp"
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

int main(int argc, char **argv) {
    try {
        if (argc != 3) throw std::runtime_error("usage: board_center_validator MODEL.mud IMAGE_LIST.txt");
        dart::NpuConfig config;
        config.enabled = config.required = config.center_only = true;
        config.input_size = 320;
        config.confidence_threshold = 0.3F;
        config.model_path = argv[1];
        auto validator = dart::create_yolo_pose_validator(config);
        std::ifstream input(argv[2]);
        if (!input) throw std::runtime_error("cannot read image list");
        std::string path;
        int count = 0, positives = 0;
        while (std::getline(input, path)) {
            if (path.empty()) continue;
            std::unique_ptr<maix::image::Image> frame(maix::image::load(path, maix::image::FMT_RGB888));
            if (!frame) throw std::runtime_error("cannot load image");
            auto result = validator->validate(*frame, {0, 0, frame->width(), frame->height()});
            for (std::size_t i = 1; i < result.keypoints.size(); ++i)
                if (result.keypoints[i].valid) throw std::runtime_error("unsupervised endpoint escaped mask");
            ++count;
            positives += result.valid;
            std::cout << "{\"image_index\":" << count - 1 << ",\"valid\":"
                      << (result.valid ? "true" : "false") << ",\"score\":" << result.confidence
                      << ",\"x\":" << result.keypoints[0].x << ",\"y\":" << result.keypoints[0].y << "}\n";
        }
        if (!count || !positives) throw std::runtime_error("no positive observations in probe corpus");
        std::cout << "{\"images\":" << count << ",\"positive_rois\":" << positives
                  << ",\"untrained_keypoints_masked\":true}\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
