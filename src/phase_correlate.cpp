#include <phase_correlate.hpp>

namespace october {

vec2f_t phase_correlate(std::shared_ptr<const image_t> img0, std::shared_ptr<const image_t> img1, float& response) {
    double r = 0.f;
	cv::Point2d shift = cv::phaseCorrelateRes(*img0, *img1, cv::noArray(), &r);
    response = static_cast<float>(r);
    return Eigen::Vector2f(shift.x, shift.y);
}

} // october
