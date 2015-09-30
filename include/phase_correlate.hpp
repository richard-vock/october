#ifndef _OCTOBER_PHASE_CORRELATE_HPP_
#define _OCTOBER_PHASE_CORRELATE_HPP_

#include "types.hpp"

namespace october {


vec2f_t phase_correlate(const image_t& img0, const image_t& img1, float& response);

vec2f_t phase_correlate(std::shared_ptr<const image_t> img0, std::shared_ptr<const image_t> img1, float& response);


} // october

#endif /* _OCTOBER_PHASE_CORRELATE_HPP_ */
