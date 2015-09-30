#include <transforms.hpp>

#include <utils.hpp>

namespace october {

std::shared_ptr<image_t> translated_image(std::shared_ptr<const image_t> img, const vec2f_t& offset) {
    auto shifted = make_image(img->cols, img->rows, 1.f);
	cv::Matx23f trans(1.f, 0.f, offset[0], 0.f, 1.f, offset[1]);
	cv::warpAffine(*img, *shifted, trans, cv::Size(img->cols, img->rows), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0.f);
    return shifted;
}

std::shared_ptr<image_t> rotated_image(std::shared_ptr<const image_t> img, float theta, const vec2f_t& center) {
    auto result = make_image(img->cols, img->rows, 1.f);
    float cos_theta = std::cos(theta);
    float sin_theta = std::sin(theta);
    vec2f_t rc;
    rc << (cos_theta * center[0] - sin_theta * center[1]), (sin_theta * center[0] + cos_theta * center[1]);
	cv::Matx23f trans(cos_theta, -sin_theta, -rc[0] + center[0], sin_theta, cos_theta, -rc[1] + center[1]);
	//cv::warpAffine(*img, *result, trans, cv::Size(img->cols, img->rows), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0.f);
	cv::warpAffine(*img, *result, trans, cv::Size(img->cols, img->rows), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, 0.f);
    return result;
}

affine2f_t origin_rotation(float rad) {
    affine2f_t transform;
    transform = rotation2f_t(-rad);
    return transform;
}

affine2f_t center_rotation(std::shared_ptr<const image_t> img, float rad) {
    affine2f_t transform;
    vec2f_t center(img->cols, img->rows);
    transform = translation2f_t(0.5f * center) * rotation2f_t(-rad) * translation2f_t(-0.5f * center);
    return transform;
}

std::shared_ptr<image_t> affine_transformed_image(std::shared_ptr<const image_t> img, const affine2f_t& transform, float background) {
    auto result = make_image(img->cols, img->rows, 1.f);
    mat3f_t m = transform.inverse().matrix();
	cv::Matx23f trans(m(0,0), m(0,1), m(0,2), m(1,0), m(1,1), m(1,2));
	cv::warpAffine(*img, *result, trans, cv::Size(img->cols, img->rows), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, background);
    return result;
}

void affine_transform_image(std::shared_ptr<image_t> img, const affine2f_t& transform, float background) {
    auto transformed = affine_transformed_image(img, transform, background);
    *img = *transformed;
}

Eigen::Matrix3f image_transform(const bbox2f_t& bbox, uint32_t width, uint32_t height, uint32_t border) {
    affine2f_t transform;

    // to unit cube
    //transform = translation2f_t(-bbox.min());
    transform = translation2f_t(-bbox.center());
    float bbox_range = sqrtf(2.f) * (bbox.max() - bbox.min()).maxCoeff();
    transform = Eigen::Scaling(1.f / bbox_range) * transform;

    // mirror Y
    transform = Eigen::Scaling(1.f, -1.f) * transform;
    //transform = translation2f_t(0.f, 1.f) * transform;

    vec2f_t sizes(static_cast<float>(width), static_cast<float>(height));
    vec2f_t borders = vec2f_t::Constant(border);

    // to image
    transform = Eigen::Scaling(sizes - 2.f * borders) * transform;
    //transform = translation2f_t(borders) * transform;
    transform = translation2f_t(0.5f * sizes) * transform;
    return transform.matrix();
}

vec2f_t image_transform_1d(float min_value, float max_value, uint32_t height, uint32_t border) {
    float range = max_value - min_value;
    float center = min_value + 0.5f * range;

    //transform = Eigen::Translation<float, 1>(-center);
    //transform = Eigen::Scaling<float, 1>(1.f / range) * transform;
    //transform = Eigen::Scaling<float, 1>(-1.f) * transform;
    //transform = Eigen::Scaling<float, 1>(height - 2 * border) * transform;
    //transform = Eigen::Translation<float, 1>(0.5f * height) * transform;
    //return transform.matrix();

    float scale = static_cast<float>(border * height) - 0.5f * static_cast<float>(height * height);
    float shift = -scale * center * range;
    scale *= 1.f / range;
    return vec2f_t(scale, shift);
}

std::shared_ptr<image_t> to_log_polar(std::shared_ptr<const image_t> img, vec2f_t center, uint32_t width, uint32_t height) {
    CvPoint2D32f c;
    c.x = center[0];
    c.y = center[1];
    IplImage tmp = *img;
    IplImage* out = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, img->channels());
    cvLogPolar(&tmp, out, c, 1.0);

    std::shared_ptr<image_t> result(new image_t(out, true));
    return result;
}

} // october
