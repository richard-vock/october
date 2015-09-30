#ifndef _OCTOBER_UTILS_HPP_
#define _OCTOBER_UTILS_HPP_

#include "types.hpp"

namespace october {


std::shared_ptr<image_t> make_image(uint32_t width, uint32_t height, float default_value);

std::shared_ptr<image_t> distance_transform(std::shared_ptr<const image_t> img);

std::shared_ptr<image_t> add_border(std::shared_ptr<const image_t> img, uint32_t top, uint32_t bottom, uint32_t left, uint32_t right, float bg);

void pca(const std::vector<vec3f_t>& points, mat3f_t& components, vec3f_t& centroid);

std::vector<mat4f_t> base_rotations();

} // october

#endif /* _OCTOBER_UTILS_HPP_ */
