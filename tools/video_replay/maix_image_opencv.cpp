#include "maix_image.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace maix {
namespace image {
namespace {

constexpr double kPi = 3.14159265358979323846;

int clamp_int(int value, int minimum, int maximum)
{
    return std::max(minimum, std::min(maximum, value));
}

class DisjointSet {
public:
    explicit DisjointSet(int size)
        : parent_(static_cast<std::size_t>(size)),
          rank_(static_cast<std::size_t>(size), 0)
    {
        std::iota(parent_.begin(), parent_.end(), 0);
    }

    int find(int value)
    {
        if (parent_[static_cast<std::size_t>(value)] != value) {
            parent_[static_cast<std::size_t>(value)] =
                find(parent_[static_cast<std::size_t>(value)]);
        }
        return parent_[static_cast<std::size_t>(value)];
    }

    void unite(int first, int second)
    {
        first = find(first);
        second = find(second);
        if (first == second) {
            return;
        }
        if (rank_[static_cast<std::size_t>(first)] <
            rank_[static_cast<std::size_t>(second)]) {
            std::swap(first, second);
        }
        parent_[static_cast<std::size_t>(second)] = first;
        if (rank_[static_cast<std::size_t>(first)] ==
            rank_[static_cast<std::size_t>(second)]) {
            ++rank_[static_cast<std::size_t>(first)];
        }
    }

private:
    std::vector<int> parent_;
    std::vector<int> rank_;
};

bool rectangles_within_margin(const cv::Rect &first,
                              const cv::Rect &second,
                              int margin)
{
    return first.x <= second.x + second.width + margin &&
           second.x <= first.x + first.width + margin &&
           first.y <= second.y + second.height + margin &&
           second.y <= first.y + first.height + margin;
}

struct Component {
    int label = 0;
    cv::Rect bounds;
    int pixels = 0;
    double center_x = 0.0;
    double center_y = 0.0;
};

}  // namespace

Blob::Blob(int x,
           int y,
           int width,
           int height,
           float center_x,
           float center_y,
           int pixels,
           float roundness)
    : x_(x),
      y_(y),
      width_(width),
      height_(height),
      center_x_(center_x),
      center_y_(center_y),
      pixels_(pixels),
      roundness_(roundness)
{
}

int Blob::x() { return x_; }
int Blob::y() { return y_; }
int Blob::w() { return width_; }
int Blob::h() { return height_; }
float Blob::cxf() { return center_x_; }
float Blob::cyf() { return center_y_; }
int Blob::pixels() { return pixels_; }
int Blob::area() { return width_ * height_; }
float Blob::density()
{
    return area() > 0 ? static_cast<float>(pixels_) / area() : 0.0F;
}
float Blob::roundness() { return roundness_; }
int Blob::perimeter() { return 2 * (width_ + height_); }

Image::Image(const cv::Mat &rgb888)
{
    if (rgb888.empty() || rgb888.type() != CV_8UC3) {
        throw std::invalid_argument("OpenCV Image adapter requires a CV_8UC3 RGB frame");
    }
    rgb888_ = rgb888.isContinuous() ? rgb888 : rgb888.clone();
}

Image::Image(int width, int height, Format format)
{
    if (width <= 0 || height <= 0 || format != Format::FMT_RGB888) {
        throw std::invalid_argument("OpenCV Image allocation requires positive RGB888 dimensions");
    }
    rgb888_.create(height, width, CV_8UC3);
}

Format Image::format() { return Format::FMT_RGB888; }
int Image::width() { return rgb888_.cols; }
int Image::height() { return rgb888_.rows; }
int Image::data_size()
{
    return static_cast<int>(rgb888_.total() * rgb888_.elemSize());
}
void *Image::data() { return rgb888_.data; }

std::vector<Blob> Image::find_blobs(
    std::vector<std::vector<int>> thresholds,
    bool invert,
    std::vector<int> roi,
    int x_stride,
    int y_stride,
    int area_threshold,
    int pixels_threshold,
    bool merge,
    int margin,
    int x_hist_bins_max,
    int y_hist_bins_max)
{
    (void)x_stride;
    (void)y_stride;
    (void)x_hist_bins_max;
    (void)y_hist_bins_max;

    if (thresholds.empty()) {
        return {};
    }

    cv::Mat lab;
    cv::cvtColor(rgb888_, lab, cv::COLOR_RGB2Lab);
    cv::Mat mask = cv::Mat::zeros(rgb888_.size(), CV_8UC1);
    for (const auto &threshold : thresholds) {
        if (threshold.size() != 6U) {
            throw std::invalid_argument("LAB threshold must contain six values");
        }
        const int l_min = clamp_int(
            static_cast<int>(std::floor(threshold[0] * 255.0 / 100.0)), 0, 255);
        const int l_max = clamp_int(
            static_cast<int>(std::ceil(threshold[1] * 255.0 / 100.0)), 0, 255);
        const int a_min = clamp_int(threshold[2] + 128, 0, 255);
        const int a_max = clamp_int(threshold[3] + 128, 0, 255);
        const int b_min = clamp_int(threshold[4] + 128, 0, 255);
        const int b_max = clamp_int(threshold[5] + 128, 0, 255);

        cv::Mat threshold_mask;
        cv::inRange(lab,
                    cv::Scalar(l_min, a_min, b_min),
                    cv::Scalar(l_max, a_max, b_max),
                    threshold_mask);
        cv::bitwise_or(mask, threshold_mask, mask);
    }
    if (invert) {
        cv::bitwise_not(mask, mask);
    }

    if (!roi.empty()) {
        if (roi.size() != 4U) {
            throw std::invalid_argument("ROI must contain x, y, width and height");
        }
        const cv::Rect image_bounds(0, 0, width(), height());
        const cv::Rect requested(roi[0], roi[1], roi[2], roi[3]);
        const cv::Rect clipped = requested & image_bounds;
        cv::Mat roi_mask = cv::Mat::zeros(mask.size(), mask.type());
        if (clipped.area() > 0) {
            mask(clipped).copyTo(roi_mask(clipped));
        }
        mask = std::move(roi_mask);
    }

    cv::Mat labels;
    cv::Mat statistics;
    cv::Mat centroids;
    const int label_count = cv::connectedComponentsWithStats(
        mask, labels, statistics, centroids, 8, CV_32S);

    std::vector<Component> components;
    for (int label = 1; label < label_count; ++label) {
        const int pixels = statistics.at<int>(label, cv::CC_STAT_AREA);
        const cv::Rect bounds(
            statistics.at<int>(label, cv::CC_STAT_LEFT),
            statistics.at<int>(label, cv::CC_STAT_TOP),
            statistics.at<int>(label, cv::CC_STAT_WIDTH),
            statistics.at<int>(label, cv::CC_STAT_HEIGHT));
        if (pixels < pixels_threshold || bounds.area() < area_threshold) {
            continue;
        }
        components.push_back({label,
                              bounds,
                              pixels,
                              centroids.at<double>(label, 0),
                              centroids.at<double>(label, 1)});
    }
    if (components.empty()) {
        return {};
    }

    DisjointSet groups(static_cast<int>(components.size()));
    if (merge) {
        const int safe_margin = std::max(0, margin);
        for (std::size_t first = 0; first < components.size(); ++first) {
            for (std::size_t second = first + 1;
                 second < components.size();
                 ++second) {
                if (rectangles_within_margin(components[first].bounds,
                                             components[second].bounds,
                                             safe_margin)) {
                    groups.unite(static_cast<int>(first),
                                 static_cast<int>(second));
                }
            }
        }
    }

    std::map<int, std::vector<int>> members;
    for (std::size_t index = 0; index < components.size(); ++index) {
        members[groups.find(static_cast<int>(index))].push_back(
            static_cast<int>(index));
    }

    std::vector<Blob> blobs;
    blobs.reserve(members.size());
    for (const auto &entry : members) {
        cv::Rect bounds;
        int pixel_count = 0;
        double weighted_x = 0.0;
        double weighted_y = 0.0;
        bool first_member = true;
        for (const int member : entry.second) {
            const Component &component = components[static_cast<std::size_t>(member)];
            bounds = first_member ? component.bounds : bounds | component.bounds;
            first_member = false;
            pixel_count += component.pixels;
            weighted_x += component.center_x * component.pixels;
            weighted_y += component.center_y * component.pixels;
        }

        cv::Mat group_mask = cv::Mat::zeros(bounds.size(), CV_8UC1);
        for (const int member : entry.second) {
            const int label = components[static_cast<std::size_t>(member)].label;
            cv::Mat label_mask;
            cv::compare(labels(bounds), label, label_mask, cv::CMP_EQ);
            cv::bitwise_or(group_mask, label_mask, group_mask);
        }
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(group_mask, contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);
        double perimeter = 0.0;
        for (const auto &contour : contours) {
            perimeter += cv::arcLength(contour, true);
        }
        const float roundness = perimeter > 0.0
                                    ? static_cast<float>(std::min(
                                          1.0,
                                          4.0 * kPi * pixel_count /
                                              (perimeter * perimeter)))
                                    : 1.0F;
        blobs.emplace_back(bounds.x,
                           bounds.y,
                           bounds.width,
                           bounds.height,
                           static_cast<float>(weighted_x / pixel_count),
                           static_cast<float>(weighted_y / pixel_count),
                           pixel_count,
                           roundness);
    }

    std::sort(blobs.begin(), blobs.end(), [](Blob &first, Blob &second) {
        return first.pixels() > second.pixels();
    });
    return blobs;
}

}  // namespace image
}  // namespace maix
