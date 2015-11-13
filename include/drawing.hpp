#ifndef _OCTOBER_DRAWING_HPP_
#define _OCTOBER_DRAWING_HPP_

#include <functional>

#include "types.hpp"

namespace october {


typedef cv::Mat image_t;

void draw_lines(std::shared_ptr<image_t> img, const std::vector<vec2i_t>& lines, float color, int thickness, std::function<bool (uint32_t)> accept_lines = [] (uint32_t) {return true;});

void draw_lines(std::shared_ptr<image_t> img, const std::vector<vec2f_t>& lines, float color, int thickness, const mat3f_t& image_transform, std::function<bool (uint32_t)> accept_lines = [] (uint32_t) {return true;});

void draw_lines(std::shared_ptr<image_t> img, const std::vector<vec2f_t>& lines, float color, int thickness, uint32_t width, uint32_t height, uint32_t border, std::function<bool (uint32_t)> accept_lines = [] (uint32_t) {return true;});

void gaussian_blur(std::shared_ptr<image_t> img, double sigma);

} // october

#endif /* _OCTOBER_DRAWING_HPP_ */
