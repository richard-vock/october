#include <registration.hpp>

#include <utils.hpp>
#include <drawing.hpp>
#include <transforms.hpp>
#include <phase_correlate.hpp>
#include <greedy_cluster.hpp>
#include <io.hpp>

namespace october {

std::tuple<bbox2f_t, vec2f_t, vec2f_t>
normalize_lines_2d_(std::vector<vec2f_t>& lines_2d_0, std::vector<vec2f_t>& lines_2d_1) {
    // compute 2d bounding boxes
    bbox2f_t bb_a, bb_b;
    for (const vec2f_t& pos : lines_2d_0) {
        bb_a.extend(pos);

    }
    for (const vec2f_t& pos : lines_2d_1) {
        bb_b.extend(pos);
    }
    vec2f_t center_0 = bb_a.center();
    vec2f_t center_1 = bb_b.center();

    // center lines
    bbox2f_t merged_bbox;
    for (vec2f_t& pos : lines_2d_0) {
        pos -= center_0;
        merged_bbox.extend(pos);
    }
    for (vec2f_t& pos : lines_2d_1) {
        pos -= center_1;
        merged_bbox.extend(pos);
    }

    return std::make_tuple(merged_bbox, center_0, center_1);
}

void
project_line_representations_(const mat4f_t& projection_0, const mat4f_t& projection_1, const std::vector<vec3f_t>& input_0, const std::vector<vec3f_t>& input_1, std::vector<vec2f_t>& output_0, std::vector<vec2f_t>& output_1) {
    // project lines to 2D space
    output_0.resize(input_0.size());
    output_1.resize(input_1.size());
    std::transform(input_0.begin(), input_0.end(), output_0.begin(), [&] (const vec3f_t& pos) { return (projection_0 * pos.homogeneous()).head(2); });
    std::transform(input_1.begin(), input_1.end(), output_1.begin(), [&] (const vec3f_t& pos) { return (projection_1 * pos.homogeneous()).head(2); });
}

vec2f_t
primary_line_direction_(const std::vector<vec2f_t>& lines, float cos_threshold) {
    typedef std::vector<vec2f_t> cluster_t;
    std::vector<vec2f_t> directions(lines.size() / 2);
    for (uint32_t i = 0; i < lines.size() / 2; ++i) {
        directions[i] = (lines[i*2+1] - lines[i*2]).normalized();
    }
    std::vector<cluster_t> clusters = greedy_cluster(
        directions,
        // check if dir belongs to cluster
        [&] (const vec2f_t& dir, const cluster_t& cluster) {
            bool included = true;
            for (const vec2f_t& d : cluster) {
                if (1.f - fabs(d.dot(dir)) > cos_threshold) {
                    included = false;
                    break;
                }
            }
            return included;
        },
        // add direction to cluster
        [&] (const vec2f_t& dir, cluster_t& cluster) {
            return cluster.push_back(dir);
        },
        // create singleton cluster from direction
        [&] (const vec2f_t& dir) {
            return cluster_t(1, dir);
        }
    );
    auto max_cluster_iter = std::max_element(clusters.begin(), clusters.end(), [&] (const cluster_t& a, const cluster_t& b) { return a.size() < b.size(); });

    vec2f_t primary_dir = vec2f_t::Zero();
    vec2f_t delta;
    uint32_t n = 0;
    for (const vec2f_t& dir : *max_cluster_iter) {
        ++n;
        delta = dir - primary_dir;
        primary_dir += delta / static_cast<float>(n);
    }
    return primary_dir.normalized();
}

mat4f_t align_z_1d(const std::vector<std::pair<float, bool>>& z_values_0, const std::vector<std::pair<float, bool>>& z_values_1) {
    /*
    std::cout << "z's 0" << "\n";
    for (const auto& z : z_values_0) {
        std::cout << z << "\n";
    }
    std::cout << "z's 1" << "\n";
    for (const auto& z : z_values_1) {
        std::cout << z << "\n";
    }
    */
    float min_z = std::min(*std::min_element(z_values_0.begin(), z_values_0.end()), *std::min_element(z_values_1.begin(), z_values_1.end())).first;
    float max_z = std::max(*std::max_element(z_values_0.begin(), z_values_0.end()), *std::max_element(z_values_1.begin(), z_values_1.end())).first;
    //vec2f_t img_transform = image_transform_1d(min_z, max_z, 2000, 200);
    Eigen::AlignedBox<float, 1> z_bbox;
    z_bbox.min()[0] = min_z;
    z_bbox.max()[0] = max_z;
    Eigen::Matrix<int, 1, 1> z_img_size;
    z_img_size << 2000;
    auto img_transform = affine_image_transform(z_bbox, z_img_size, 500);

    std::shared_ptr<image_t> img_0 = std::make_shared<image_t>(2000, 400, CV_32FC1, cv::Scalar::all(0.f));
    std::shared_ptr<image_t> img_1 = std::make_shared<image_t>(2000, 400, CV_32FC1, cv::Scalar::all(0.f));
    
    int zTrue = 0;
    int zFalse = 0;
    
    for (const auto& z : z_values_0) {
        uint32_t i = static_cast<int>((img_transform * vec2f_t(z.first, 1.f))[0]);
        int min_j = z.second ? 0 : 200;
        int max_j = z.second ? 200 : 400;
        if (z.second) ++zTrue;
        if (!z.second) ++zFalse;
        for (int j = min_j; j < max_j; ++j) {
            for (int d = -1; d <= 1; ++d) {
                img_0->at<float>(i+d, j) = 1.f;
            }
        }
    }
    for (const auto& z : z_values_1) {
        uint32_t i = static_cast<int>((img_transform * vec2f_t(z.first, 1.f))[0]);
        int min_j = z.second ? 0 : 200;
        int max_j = z.second ? 200 : 400;
        if (z.second) ++zTrue;
        if (!z.second) ++zFalse;
        for (int j = min_j; j < max_j; ++j) {
            for (int d = -1; d <= 1; ++d) {
                img_1->at<float>(i+d, j) = 1.f;
            }
        }
    }
    
    std::cout << zTrue << ", " << zFalse << std::endl;
    
    gaussian_blur(img_0, 2.0);
    gaussian_blur(img_1, 2.0);
    
	cv::Mat png_0(2000, 50, CV_16UC1), png_1(2000, 50, CV_16UC1);
	img_0->convertTo(png_0, CV_16UC1, 65535.0);
	img_1->convertTo(png_1, CV_16UC1, 65535.0);
	cv::imwrite("/tmp/a.png", png_0);
	cv::imwrite("/tmp/b.png", png_1);
    float response = 0.f;
    vec2f_t shift = phase_correlate(*img_0, *img_1, response);
    std::cout << "Z shift: " << shift.transpose() << "\n";

    std::cout << "img transform:" << std::endl;
    std::cout << img_transform << std::endl << std::endl;

    mat4f_t transform = mat4f_t::Identity();
    transform(2, 3) = shift[1] / img_transform(0, 0);
    
    return transform;
}

mat4f_t
align_lines_2d(std::vector<vec2f_t>& lines_2d_0, std::vector<vec2f_t>& lines_2d_1, float* response = nullptr) {
    vec2f_t center_0, center_1;
    bbox2f_t bbox;
    std::tie(bbox, center_0, center_1) = normalize_lines_2d_(lines_2d_0, lines_2d_1);

    mat3f_t img_transform = image_transform(bbox, 2000, 2000, 100);
    mat3f_t inv_img_transform = img_transform.inverse();

    vec2f_t primary_0 = primary_line_direction_(lines_2d_0, 0.05f);
    vec2f_t primary_1 = primary_line_direction_(lines_2d_1, 0.05f);

    std::shared_ptr<image_t> img_0 = make_image(2000, 2000, 0.f);
    std::shared_ptr<image_t> img_1 = make_image(2000, 2000, 0.f);
    draw_lines(img_0, lines_2d_0, 1.f, 5, img_transform);
    draw_lines(img_1, lines_2d_1, 1.f, 5, img_transform);

    float primary_angle_0 = std::atan2(primary_0[1], primary_0[0]);
    float primary_angle_1 = std::atan2(primary_1[1], primary_1[0]);

    vec2f_t img_center(static_cast<float>(img_0->cols) / 2.f, static_cast<float>(img_0->rows) / 2.f);
    std::shared_ptr<image_t> rot_b = affine_transformed_image(img_1, center_rotation(img_1, -primary_angle_1), 0.f);

    affine2f_t final_transform;
    float max_response = 0.f;
    for (uint32_t i = 0; i < 4; ++i) {
        // determine rotation by multiples of 90°
        float theta = -primary_angle_0 + static_cast<float>(i) * static_cast<float>(0.5 * M_PI);
        affine2f_t rotation;
        rotation = center_rotation(img_0, theta);

        // rotate image
        std::shared_ptr<image_t> rot_a = affine_transformed_image(img_0, rotation, 0.f);
        //write_image(rot_a, "rotated_a_"+std::to_string(i)+".png");

        float response;
        vec2f_t shift = phase_correlate(rot_a, rot_b, response);

        if (response > max_response) {
            final_transform = inv_img_transform * center_rotation(img_1, primary_angle_1) * translation2f_t(shift) * rotation * img_transform;
            auto final_img = affine_transformed_image(img_0, center_rotation(img_1, primary_angle_1) * translation2f_t(shift) * rotation, 0.f);
            //write_image(final_img, "/tmp/a.png");
            max_response = response;
        }
    }
    //write_image(img_1, "/tmp/b.png");

    mat4f_t local_transform = mat4f_t::Identity(), center_mat_0 = mat4f_t::Identity(), center_mat_1 = mat4f_t::Identity();
    local_transform.block<2,2>(0, 0) = final_transform.linear();
    local_transform.block<2,1>(0, 3) = final_transform.translation();
    center_mat_0.block<2,1>(0, 3) = -center_0;
    center_mat_1.block<2,1>(0, 3) =  center_1;

    if (response) *response = max_response;

    return center_mat_1 * local_transform * center_mat_0;
}

mat4f_t
align_lines_2d_translation(std::vector<vec2f_t>& lines_2d_0, std::vector<vec2f_t>& lines_2d_1, float* response = nullptr, uint32_t rot_idx = 0) {
    vec2f_t center_0, center_1;
    bbox2f_t bbox;
    std::tie(bbox, center_0, center_1) = normalize_lines_2d_(lines_2d_0, lines_2d_1);

    mat3f_t img_transform = image_transform(bbox, 2000, 2000, 100);
    mat3f_t inv_img_transform = img_transform.inverse();

    std::shared_ptr<image_t> img_0 = make_image(2000, 2000, 0.f);
    std::shared_ptr<image_t> img_1 = make_image(2000, 2000, 0.f);
    draw_lines(img_0, lines_2d_0, 1.f, 5, img_transform);
    draw_lines(img_1, lines_2d_1, 1.f, 5, img_transform);
    
    gaussian_blur(img_0, 10.0);
    gaussian_blur(img_1, 10.0);
    
    write_image(img_0, "/tmp/img_0_" + std::to_string(rot_idx) + ".png");
    write_image(img_1, "/tmp/img_1_" + std::to_string(rot_idx) + ".png");

    float signal_response;
    vec2f_t shift = phase_correlate(img_0, img_1, signal_response);
    mat3f_t shift_mat = mat3f_t::Identity();
    shift_mat.block<2,1>(0, 2) = shift;

    affine2f_t final_transform;
    final_transform = inv_img_transform * shift_mat * img_transform;
    if (response) *response = signal_response;

    mat4f_t local_transform = mat4f_t::Identity(), center_mat_0 = mat4f_t::Identity(), center_mat_1 = mat4f_t::Identity();
    local_transform.block<2,2>(0, 0) = final_transform.linear();
    local_transform.block<2,1>(0, 3) = final_transform.translation();
    center_mat_0.block<2,1>(0, 3) = -center_0;
    center_mat_1.block<2,1>(0, 3) =  center_1;

    return center_mat_1 * local_transform * center_mat_0;
}

mat4f_t
align_lines_2d(const mat4f_t& projection_0, const mat4f_t& projection_1, const std::vector<vec3f_t>& lines_0, const std::vector<vec3f_t>& lines_1) {
    std::vector<vec2f_t> lines_2d_0, lines_2d_1;
    project_line_representations_(projection_0, projection_1, lines_0, lines_1, lines_2d_0, lines_2d_1);

    return projection_1.inverse() * align_lines_2d(lines_2d_0, lines_2d_1) * projection_0;
}

mat4f_t
align_lines_2d(const vec3f_t& up_0, const vec3f_t& up_1, const std::vector<vec3f_t>& lines_0, const std::vector<vec3f_t>& lines_1) {
    mat4f_t projection_0 = mat4f_t::Identity(), projection_1 = mat4f_t::Identity();

    vec3f_t z_0 = up_0.normalized();
    vec3f_t z_1 = up_1.normalized();

    // make linear part of projection matrices a projection into some orthonormal base with the third base vector being the up direction
    projection_0.block<1,3>(2, 0) = z_0.transpose();
    projection_1.block<1,3>(2, 0) = z_1.transpose();

    // tangents are irrelevant if they at least are othogonal
    projection_0.block<1,3>(1, 0) = z_0.unitOrthogonal().transpose();
    projection_1.block<1,3>(1, 0) = z_1.unitOrthogonal().transpose();

    // bitangent is computed by the cross product
    projection_0.block<1,3>(0, 0) = projection_0.block<1,3>(1, 0).cross(projection_0.block<1,3>(0, 0)).normalized();
    projection_1.block<1,3>(0, 0) = projection_1.block<1,3>(1, 0).cross(projection_1.block<1,3>(0, 0)).normalized();

    return align_lines_2d(projection_0, projection_1, lines_0, lines_1);
}

mat4f_t align_3d(const std::vector<bounded_plane::ptr_t>& planes_0, const std::vector<bounded_plane::ptr_t>& planes_1) {
    mat3f_t primary_0 = primary_normal_directions(planes_0, 0.01f);
    mat3f_t primary_1 = primary_normal_directions(planes_1, 0.01f);
    mat4f_t normalization_0 = mat4f_t::Identity();
    mat4f_t normalization_1 = mat4f_t::Identity();
    normalization_0.block<3,3>(0,0) = primary_0.transpose();
    normalization_1.block<3,3>(0,0) = primary_1.transpose();

    std::vector<mat4f_t> rotations = base_rotations();
    float max_response = 0.f;
    mat4f_t local_transform;
    uint32_t rot_idx = 0;
    for (const auto& rotation : rotations) {
        mat4f_t pre_0 = rotation.transpose() * normalization_0;

        // project lines
        std::vector<vec2f_t> lines_2d_0, lines_2d_1;
        for (auto p : planes_0) {
            if (!p->is_parallel_to(pre_0, vec3f_t::UnitZ(), 0.01f)) continue;
            vec2f_t src, tgt;
            //p->transform(projection_0);
            std::tie(src, tgt) = p->project_2d_line(pre_0, vec3f_t::UnitZ());
            lines_2d_0.push_back(src);
            lines_2d_0.push_back(tgt);
        }
        for (auto p : planes_1) {
            if (!p->is_parallel_to(normalization_1, vec3f_t::UnitZ(), 0.01f)) continue;
            vec2f_t src, tgt;
            //p->transform(projection_0);
            std::tie(src, tgt) = p->project_2d_line(normalization_1, vec3f_t::UnitZ());
            lines_2d_1.push_back(src);
            lines_2d_1.push_back(tgt);
        }

        if (lines_2d_0.empty() || lines_2d_1.empty()) {
            continue;
        }

        float response;
        mat4f_t align_xy = align_lines_2d_translation(lines_2d_0, lines_2d_1, &response, rot_idx);
        if (response > max_response) {
            local_transform = align_xy * rotation.transpose();
            max_response = response;
            std::cout << rotation.transpose() << "\n";
            std::cout << "Best rot idx: " << rot_idx << std::endl;
        }
        
        ++rot_idx;
    }

    return normalization_1.inverse() * local_transform * normalization_0;
}

mat4f_t
align(const std::vector<bounded_plane::ptr_t>& planes_0, const std::vector<bounded_plane::ptr_t>& planes_1, const vec3f_t& up_0, const vec3f_t& up_1) {
    mat4f_t projection_0 = mat4f_t::Identity(), projection_1 = mat4f_t::Identity();

    //vec3f_t z_0 = up_0.normalized();
    //vec3f_t z_1 = up_1.normalized();

    // make linear part of projection matrices a projection into some orthonormal base with the third base vector being the up direction
    //projection_0.block<1,3>(2, 0) = z_0.transpose();
    //projection_1.block<1,3>(2, 0) = z_1.transpose();

    // tangents are irrelevant if they at least are orthogonal
    //projection_0.block<1,3>(1, 0) = z_0.unitOrthogonal().transpose();
    //projection_1.block<1,3>(1, 0) = z_1.unitOrthogonal().transpose();

    // bitangent is computed by the cross product
    //projection_0.block<1,3>(0, 0) = projection_0.block<1,3>(1, 0).cross(projection_0.block<1,3>(0, 0)).normalized();
    //projection_1.block<1,3>(0, 0) = projection_1.block<1,3>(1, 0).cross(projection_1.block<1,3>(0, 0)).normalized();

    /*
    std::vector<vec2f_t> lines_2d_0, lines_2d_1;
    for (const auto& p : planes_0) {
        if (!p->is_parallel_to(up_0, 0.01f)) continue;
        vec2f_t src, tgt;
        //p->transform(projection_0);
        std::tie(src, tgt) = p->project_2d_line(vec3f_t::UnitZ());
        lines_2d_0.push_back(src);
        lines_2d_0.push_back(tgt);
    }
    for (const auto& p : planes_1) {
        if (!p->is_parallel_to(up_1, 0.01f)) continue;
        vec2f_t src, tgt;
        //p->transform(projection_1);
        std::tie(src, tgt) = p->project_2d_line(vec3f_t::UnitZ());
        lines_2d_1.push_back(src);
        lines_2d_1.push_back(tgt);
    }
    
    if (lines_2d_0.empty() || lines_2d_1.empty()) {
        throw std::runtime_error("Unable to align (lines empty): Wrong up direction?");
    }

    auto align_xy = align_lines_2d(lines_2d_0, lines_2d_1);
    */
    
    auto align_xy = align_3d(planes_0, planes_1);

    std::vector<std::pair<float, bool>> z_values_0, z_values_1;
    for (const auto& p : planes_0) {
        vec3f_t normal = align_xy.topLeftCorner<3, 3>() * p->normal();
        if (1.f - fabs(normal.dot(up_0)) < 0.01f) {
            vec3f_t origin = (align_xy * p->origin().homogeneous()).head(3);
            z_values_0.push_back(std::make_pair(origin.dot(up_0), normal[2] > 0.f));
        }
    }
    for (const auto& p : planes_1) {
        vec3f_t normal = align_xy.topLeftCorner<3, 3>() * p->normal();
        if (1.f - fabs(normal.dot(up_1)) < 0.01f) {
            vec3f_t origin = (align_xy * p->origin().homogeneous()).head(3);
            z_values_1.push_back(std::make_pair(origin.dot(up_1), normal[2] > 0.f));
        }
    }
    
    if (z_values_0.empty() || z_values_1.empty()) {
        throw std::runtime_error("Unable to align (z values empty): Wrong up direction?");
    }

    auto align_z = align_z_1d(z_values_0, z_values_1);
    
    std::cout << "align_z = " << align_z << std::endl;

    return projection_1.inverse() * align_z * align_xy * projection_0;
}


} // october
