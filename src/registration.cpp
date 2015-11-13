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

std::tuple<bbox2f_t, std::vector<vec2f_t>, std::vector<vec2f_t>>
normalize_lines_2d_(std::vector<std::vector<vec2f_t>>& lines_2d_vec_0, std::vector<std::vector<vec2f_t>>& lines_2d_vec_1, bool center = true) {
    std::vector<vec2f_t> centers_0, centers_1;
    
    for (const auto& lines_2d_0 : lines_2d_vec_0) {
        bbox2f_t bb;
        for (const vec2f_t& pos : lines_2d_0) {
            bb.extend(pos);
        }
        centers_0.push_back(bb.center());
    }
    
    for (const auto& lines_2d_1 : lines_2d_vec_1) {
        bbox2f_t bb;
        for (const vec2f_t& pos : lines_2d_1) {
            bb.extend(pos);
        }
        centers_1.push_back(bb.center());
    }
    
    bbox2f_t merged_bbox;
    
    uint32_t i = 0;
    for (auto& lines_2d_0 : lines_2d_vec_0) {
        for (vec2f_t& pos : lines_2d_0) {
            //if (center) pos -= centers_0[i];
            merged_bbox.extend(pos);
        }
        ++i;
    }
    
    i = 0;
    for (auto& lines_2d_1 : lines_2d_vec_1) {
        for (vec2f_t& pos : lines_2d_1) {
            //if (center) pos -= centers_1[i];
            merged_bbox.extend(pos);
        }
        ++i;
    }
    
    return std::make_tuple(merged_bbox, centers_0, centers_1);
}

mat4f_t
align_lines_2d_translation_(std::vector<vec2f_t>& lines_2d_0, std::vector<vec2f_t>& lines_2d_1, float* response = nullptr, uint32_t rot_idx = 0) {
    vec2f_t center_0, center_1;
    bbox2f_t bbox;
    std::tie(bbox, center_0, center_1) = normalize_lines_2d_(lines_2d_0, lines_2d_1);
    
    const int IMG_SIZE = 4096;
    const int IMG_BORDER = 500;
    const int LINE_WIDTH = 3;
    const float SMOOTHING_SIGMA = 3.f;

    mat3f_t img_transform = image_transform(bbox, IMG_SIZE, IMG_SIZE, IMG_BORDER);
    mat3f_t inv_img_transform = img_transform.inverse();

    std::shared_ptr<image_t> img_0 = make_image(IMG_SIZE, IMG_SIZE, 0.f);
    std::shared_ptr<image_t> img_1 = make_image(IMG_SIZE, IMG_SIZE, 0.f);
    draw_lines(img_0, lines_2d_0, 1.f, LINE_WIDTH, img_transform);
    draw_lines(img_1, lines_2d_1, 1.f, LINE_WIDTH, img_transform);
    
    if (SMOOTHING_SIGMA) {
        gaussian_blur(img_0, SMOOTHING_SIGMA);
        gaussian_blur(img_1, SMOOTHING_SIGMA);
    }
    
#ifdef OCTOBER_DEBUG
    write_image(img_0, "/tmp/img_0_" + std::to_string(rot_idx) + ".png");
    write_image(img_1, "/tmp/img_1_" + std::to_string(rot_idx) + ".png");
#endif // OCTOBER_DEBUG

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

mat4f_t align_3d(const std::vector<bounded_plane::ptr_t>& planes_0, const std::vector<bounded_plane::ptr_t>& planes_1) {
    mat3f_t primary_0 = primary_normal_directions(planes_0, 0.01f, 0.01f);
    mat3f_t primary_1 = primary_normal_directions(planes_1, 0.01f, 0.01f);
    
    // Just use the first primary direction and determine the rest under the assumption that z is the up direction.
    primary_0(2, 0) = 0.f;
    primary_0.col(0).normalize();
    primary_0.col(1) = primary_0.col(0).cross(vec3f_t::UnitZ());
    primary_0.col(2) = vec3f_t::UnitZ();
    primary_1(2, 0) = 0.f;
    primary_1.col(0).normalize();
    primary_1.col(1) = primary_1.col(0).cross(vec3f_t::UnitZ());
    primary_1.col(2) = vec3f_t::UnitZ();
    
    mat4f_t normalization_0 = mat4f_t::Identity();
    mat4f_t normalization_1 = mat4f_t::Identity();
    normalization_0.block<3,3>(0,0) = primary_0.transpose();
    normalization_1.block<3,3>(0,0) = primary_1.transpose();

    std::vector<mat4f_t> rotations;
    for (int i = 0; i < 4; ++i) {
        mat3f_t rot;
        rot = Eigen::AngleAxisf((0.5 * M_PI) * i, Eigen::Vector3f::UnitZ());
        mat4f_t trf = mat4f_t::Identity();
        trf.block<3, 3>(0, 0) = rot;
        rotations.push_back(trf);
    }
    
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
        mat4f_t align_xy = align_lines_2d_translation_(lines_2d_0, lines_2d_1, &response, rot_idx);
        if (response > max_response) {
            local_transform = align_xy * rotation.transpose();
            max_response = response;
#ifdef OCTOBER_DEBUG
            std::cout << rotation.transpose() << "\n";
            std::cout << "Best rot idx: " << rot_idx << std::endl;
#endif // OCTOBER_DEBUG
        }
        
        ++rot_idx;
    }
    
    mat4f_t align_xy = normalization_1.inverse() * local_transform * normalization_0;
    
    std::vector<std::vector<vec2f_t>> lines_2d_vec_0, lines_2d_vec_1;
    std::vector<std::vector<int>>     types_2d_vec_0, types_2d_vec_1;
    
    for (int i = 0; i < 2; ++i) {
        mat3f_t rot;
        rot = Eigen::AngleAxisf(-0.5 * M_PI, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf((0.5 * M_PI) * i, Eigen::Vector3f::UnitZ());
        mat4f_t trf = mat4f_t::Identity();
        trf.block<3, 3>(0, 0) = rot;
        
        mat4f_t pre_transform_0 = trf * normalization_1 * align_xy;
        std::vector<vec2f_t> lines_2d_0;
        std::vector<int>     types_2d_0;
        for (auto p : planes_0) {
            if (!p->is_parallel_to(pre_transform_0, vec3f_t::UnitZ(), 0.01f)) continue;
            vec2f_t src, tgt;
            std::tie(src, tgt) = p->project_2d_line(pre_transform_0, vec3f_t::UnitZ());
            lines_2d_0.push_back(src);
            lines_2d_0.push_back(tgt);
            if (p->normal()[2] > 0.99f) {
                types_2d_0.push_back(0);
            } else if (p->normal()[2] < -0.99f) {
                types_2d_0.push_back(1);
            } else {
                types_2d_0.push_back(2);
            }
        }
        lines_2d_vec_0.push_back(lines_2d_0);
        types_2d_vec_0.push_back(types_2d_0);
        
        mat4f_t pre_transform_1 = trf * normalization_1;
        std::vector<vec2f_t> lines_2d_1;
        std::vector<int>     types_2d_1;
        for (auto p : planes_1) {
            if (!p->is_parallel_to(pre_transform_1, vec3f_t::UnitZ(), 0.01f)) continue;
            vec2f_t src, tgt;
            std::tie(src, tgt) = p->project_2d_line(pre_transform_1, vec3f_t::UnitZ());
            lines_2d_1.push_back(src);
            lines_2d_1.push_back(tgt);
            if (p->normal()[2] > 0.99f) {
                types_2d_1.push_back(0);
            } else if (p->normal()[2] < -0.99f) {
                types_2d_1.push_back(1);
            } else {
                types_2d_1.push_back(2);
            }
        }
        lines_2d_vec_1.push_back(lines_2d_1);
        types_2d_vec_1.push_back(types_2d_1);
    }
    
    bbox2f_t merged_bbox;
    std::vector<vec2f_t> centers_0, centers_1;
    std::tie(merged_bbox, centers_0, centers_1) = normalize_lines_2d_(lines_2d_vec_0, lines_2d_vec_1, false);
    
    const int Z_SUBIMG_SIZE = 4096;
    const int Z_SUBIMG_BORDER = 500;
    const int Z_SUBIMG_LINE_WIDTH = 3;
    const float Z_SUBIMG_SMOOTHING_SIGMA = 3.f;
    
    mat3f_t z_img_transform = image_transform(merged_bbox, Z_SUBIMG_SIZE, Z_SUBIMG_SIZE, Z_SUBIMG_BORDER);
    z_img_transform(1, 1) *= 0.5f * merged_bbox.sizes()[0] / merged_bbox.sizes()[1];
    
    std::shared_ptr<image_t> z_img_0 = make_image(2*Z_SUBIMG_SIZE, Z_SUBIMG_SIZE, 0.f);
    for (uint32_t i = 0; i < 2; ++i) {
        mat3f_t tmp_z_img_transform = z_img_transform;
        tmp_z_img_transform(0, 2) += i * Z_SUBIMG_SIZE;
        //mat3f_t inv_img_transform = img_transform.inverse();
        draw_lines(z_img_0, lines_2d_vec_0[i], 1.f, Z_SUBIMG_LINE_WIDTH, tmp_z_img_transform, [&] (uint32_t idx) {return types_2d_vec_0[i][idx] == 0 || types_2d_vec_0[i][idx] == 2;});
        //tmp_z_img_transform(0, 2) += 2*Z_SUBIMG_SIZE;
        //draw_lines(z_img_0, lines_2d_vec_0[i], 1.f, Z_SUBIMG_LINE_WIDTH, tmp_z_img_transform, [&] (uint32_t idx) {return types_2d_vec_0[i][idx] == 1 || types_2d_vec_0[i][idx] == 2;});
    }
    
    std::shared_ptr<image_t> z_img_1 = make_image(2*Z_SUBIMG_SIZE, Z_SUBIMG_SIZE, 0.f);
    for (uint32_t i = 0; i < 2; ++i) {
        mat3f_t tmp_z_img_transform = z_img_transform;
        tmp_z_img_transform(0, 2) += i * Z_SUBIMG_SIZE;
        //mat3f_t inv_img_transform = img_transform.inverse();
        draw_lines(z_img_1, lines_2d_vec_1[i], 1.f, Z_SUBIMG_LINE_WIDTH, tmp_z_img_transform, [&] (uint32_t idx) {return types_2d_vec_1[i][idx] == 0 || types_2d_vec_1[i][idx] == 2;});
        //tmp_z_img_transform(0, 2) += 2*Z_SUBIMG_SIZE;
        //draw_lines(z_img_1, lines_2d_vec_1[i], 1.f, Z_SUBIMG_LINE_WIDTH, tmp_z_img_transform, [&] (uint32_t idx) {return types_2d_vec_1[i][idx] == 1 || types_2d_vec_1[i][idx] == 2;});
    }
    
    if (Z_SUBIMG_SMOOTHING_SIGMA) {
        gaussian_blur(z_img_0, Z_SUBIMG_SMOOTHING_SIGMA);
        gaussian_blur(z_img_1, Z_SUBIMG_SMOOTHING_SIGMA);
    }
    
#ifdef OCTOBER_DEBUG
    write_image(z_img_0, "/tmp/z_img_0.png");
    write_image(z_img_1, "/tmp/z_img_1.png");
#endif // OCTOBER_DEBUG
    
    float z_signal_response;
    vec2f_t shift = phase_correlate(z_img_0, z_img_1, z_signal_response);
    
#ifdef OCTOBER_DEBUG
    std::cout << "Z shift: " << shift.transpose() << std::endl;
#endif // OCTOBER_DEBUG
    
    mat4f_t align_z = mat4f_t::Identity();
    align_z(2, 3) = shift[1] / z_img_transform(1, 1);
    
#ifdef OCTOBER_DEBUG
    std::cout << "Z align: " << align_z(2, 3) << std::endl;
#endif // OCTOBER_DEBUG
    
    /*
    mat3f_t shift_mat = mat3f_t::Identity();
    shift_mat.block<2,1>(0, 2) = shift;

    affine2f_t z_transform;
    z_transform = inv_img_transform * shift_mat * img_transform;
    if (response) *response = signal_response;

    mat4f_t local_transform = mat4f_t::Identity(), center_mat_0 = mat4f_t::Identity(), center_mat_1 = mat4f_t::Identity();
    local_transform.block<2,2>(0, 0) = final_transform.linear();
    local_transform.block<2,1>(0, 3) = final_transform.translation();
    */

    return align_z * align_xy;
}

} // october
