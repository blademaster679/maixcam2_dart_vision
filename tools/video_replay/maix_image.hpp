#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace maix {
namespace image {

enum class Format {
    FMT_RGB888 = 0,
    FMT_GRAYSCALE = 12,
};

class Blob {
public:
    Blob(int x,
         int y,
         int width,
         int height,
         float center_x,
         float center_y,
         int pixels,
         float roundness);

    int x();
    int y();
    int w();
    int h();
    float cxf();
    float cyf();
    int pixels();
    int area();
    float density();
    float roundness();
    int perimeter();

private:
    int x_ = 0;
    int y_ = 0;
    int width_ = 0;
    int height_ = 0;
    float center_x_ = 0.0F;
    float center_y_ = 0.0F;
    int pixels_ = 0;
    float roundness_ = 0.0F;
};

class Image {
public:
    explicit Image(const cv::Mat &rgb888);
    Image(int width, int height, Format format);

    Format format();
    int width();
    int height();
    int data_size();
    void *data();

    std::vector<Blob> find_blobs(
        std::vector<std::vector<int>> thresholds = {},
        bool invert = false,
        std::vector<int> roi = {},
        int x_stride = 2,
        int y_stride = 1,
        int area_threshold = 10,
        int pixels_threshold = 10,
        bool merge = false,
        int margin = 0,
        int x_hist_bins_max = 0,
        int y_hist_bins_max = 0);

private:
    cv::Mat rgb888_;
};

}  // namespace image
}  // namespace maix
