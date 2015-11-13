#ifndef REGISTRATION_HPP_
#define REGISTRATION_HPP_

#include "types.hpp"
#include "bounded_plane.hpp"

namespace october {

mat4f_t
align_3d(const std::vector<bounded_plane::ptr_t>& planes_0, const std::vector<bounded_plane::ptr_t>& planes_1);

} // october

#endif /* REGISTRATION_HPP_ */
