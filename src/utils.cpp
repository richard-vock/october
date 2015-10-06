#include <utils.hpp>
#include <regex>

namespace october {

std::shared_ptr<image_t> make_image(uint32_t width, uint32_t height, float default_value) {
    return std::make_shared<image_t>(height, width, CV_32FC1, default_value);
}

std::shared_ptr<image_t> distance_transform(std::shared_ptr<const image_t> img) {
    auto result = make_image(img->cols, img->rows, 1.f);
	cv::Mat img8(img->rows, img->cols, CV_8UC1);
	img->convertTo(img8, CV_8UC1, 255.0);
    cv::distanceTransform(img8, *result, CV_DIST_L2, CV_DIST_MASK_PRECISE);
    return result;
}

std::shared_ptr<image_t> add_border(std::shared_ptr<const image_t> img, uint32_t top, uint32_t bottom, uint32_t left, uint32_t right, float bg) {
    uint32_t new_width = img->cols + left + right;
    uint32_t new_height = img->rows + top + bottom;
    auto result = make_image(new_width, new_height, bg);
    cv::Rect rect(left, top, img->cols, img->rows);
    img->copyTo((*result)(rect));
    return result;
}

void pca(const std::vector<vec3f_t>& points, mat3f_t& components, vec3f_t& centroid) {
    // compute centroid
    centroid = vec3f_t::Zero();
    vec3f_t delta;
    uint32_t idx = 0;
    for (const auto& p : points) {
        delta = p - centroid;
        centroid += delta / static_cast<float>(++idx);
    }

    // compute PCA
    Eigen::MatrixXf pos_mat(points.size(), 3);
    for (uint32_t i = 0; i < points.size(); ++i) {
        pos_mat.row(i) = (points[i] - centroid).transpose();
    }
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(pos_mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    components = svd.matrixV();
}

std::vector<mat4f_t> base_rotations() {
    std::vector<mat4f_t> results;

    for (uint32_t i = 0; i < 3; ++i) {
        for (uint32_t j = 0; j < 3; ++j) {
            if (i == j) continue;
            for (uint32_t s = 0; s < 4; ++s) {
                float sign_i = (s / 2) ? -1.f : 1.f;
                float sign_j = (s % 2) ? -1.f : 1.f;
                results.push_back(mat4f_t::Identity());
                results.back().block<3,1>(0,0) = sign_i * vec3f_t::Unit(i);
                results.back().block<3,1>(0,1) = sign_j * vec3f_t::Unit(j);
                results.back().block<3,1>(0,2) = results.back().block<3,1>(0,0).cross(results.back().block<3,1>(0,1));
            }
        }
    }

    return results;
}

std::vector<std::string> split_string(const std::string& str, const std::string& delim_regex) {
    std::regex rgx(delim_regex);
    std::sregex_token_iterator iter(str.begin(), str.end(), rgx, -1), end;
    std::vector<std::string> tokens;
    for ( ; iter != end; ++iter) {
        tokens.push_back(*iter);
    }
    return tokens;
}

} // october
