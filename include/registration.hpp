#ifndef REGISTRATION_HPP_
#define REGISTRATION_HPP_

#include "types.hpp"

namespace october {


mat4f_t
align_lines_2d(const mat4f_t& projection_0, const mat4f_t& projection_1, const std::vector<vec3f_t>& lines_0, const std::vector<vec3f_t>& lines_1);

mat4f_t
align_lines_2d(const vec3f_t& up_0, const vec3f_t& up_1, const std::vector<vec3f_t>& lines_0, const std::vector<vec3f_t>& lines_1);


} // october

#endif /* REGISTRATION_HPP_ */
