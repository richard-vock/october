#include <drawing.hpp>

namespace october {


void draw_lines(std::shared_ptr<image_t> img, const std::vector<vec2i_t>& lines, float color, int thickness, std::function<bool (uint32_t)> accept_lines) {
    uint32_t count = lines.size() / 2;
    for (uint32_t i = 0; i < count; ++i) {
        if (!accept_lines(i)) continue;
        cv::Point2i p0(lines[i*2][0], lines[i*2][1]), p1(lines[i*2+1][0], lines[i*2+1][1]);
        cv::line(*img, p0, p1, cv::Scalar(color), thickness, 8);
    }
}

void draw_lines(std::shared_ptr<image_t> img, const std::vector<vec2f_t>& lines, float color, int thickness, const mat3f_t& image_transform, std::function<bool (uint32_t)> accept_lines) {
    std::vector<vec2i_t> discrete_lines(lines.size());
    for (uint32_t i = 0; i < lines.size(); ++i) {
        discrete_lines[i] = (image_transform * lines[i].homogeneous()).head(2).template cast<int>();
    }
    draw_lines(img, discrete_lines, color, thickness, accept_lines);
}

void draw_lines(std::shared_ptr<image_t> img, const std::vector<vec2f_t>& lines, float color, int thickness, uint32_t width, uint32_t height, uint32_t border, std::function<bool (uint32_t)> accept_lines) {
    std::vector<vec2f_t> extruded_lines(lines.size());
    std::vector<vec2i_t> discrete_lines(lines.size());
    vec2f_t dim(width - 2*border, height - 2*border);
    for (uint32_t i = 0; i < lines.size() / 2; ++i) {
        vec2f_t p0 = lines[i*2], p1 = lines[i*2+1];
        vec2f_t dir = (p1 - p0).normalized();
        p0 -= 2.f * dir;
        p1 += 2.f * dir;
        extruded_lines[i*2+0] = p0;
        extruded_lines[i*2+1] = p1;
    }
    for (uint32_t i = 0; i < lines.size(); ++i) {
        vec2f_t rescaled = (extruded_lines[i].array() * dim.array()).matrix();// + vec2f_t::Constant(border);
        discrete_lines[i] = rescaled.template cast<int>();
    }
    draw_lines(img, discrete_lines, color, thickness, accept_lines);
}

void gaussian_blur(std::shared_ptr<image_t> img, double sigma) {
    cv::GaussianBlur(*img, *img, cv::Size(cv::Point(0, 0)), sigma);
}


} // october
