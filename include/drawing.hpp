#ifndef _OCTOBER_DRAWING_HPP_
#define _OCTOBER_DRAWING_HPP_

#include "types.hpp"

namespace october {


typedef cv::Mat image_t;

void draw_lines(std::shared_ptr<image_t> img, const std::vector<vec2i_t>& lines, float color, int thickness);

void draw_lines(std::shared_ptr<image_t> img, const std::vector<vec2f_t>& lines, float color, int thickness, const mat3f_t& image_transform);

void draw_lines(std::shared_ptr<image_t> img, const std::vector<vec2f_t>& lines, float color, int thickness, uint32_t width, uint32_t height, uint32_t border);


} // october

#endif /* _OCTOBER_DRAWING_HPP_ */
