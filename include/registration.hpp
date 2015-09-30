#ifndef REGISTRATION_HPP_
#define REGISTRATION_HPP_

#include "types.hpp"
#include "bounded_plane.hpp"

namespace october {


mat4f_t
align_lines_2d(const mat4f_t& projection_0, const mat4f_t& projection_1, const std::vector<vec3f_t>& lines_0, const std::vector<vec3f_t>& lines_1);

mat4f_t
align_lines_2d(const vec3f_t& up_0, const vec3f_t& up_1, const std::vector<vec3f_t>& lines_0, const std::vector<vec3f_t>& lines_1);

mat4f_t
align_3d(const std::vector<bounded_plane::ptr_t>& planes_0, const std::vector<bounded_plane::ptr_t>& planes_1);

mat4f_t
align(const std::vector<bounded_plane::ptr_t>& planes_0, const std::vector<bounded_plane::ptr_t>& planes_1, const vec3f_t& up_0 = vec3f_t::UnitZ(), const vec3f_t& up_1 = vec3f_t::UnitZ());


} // october

#endif /* REGISTRATION_HPP_ */
