#include <registration.hpp>

#include <utils.hpp>
#include <drawing.hpp>
#include <transforms.hpp>
#include <phase_correlate.hpp>
#include <greedy_cluster.hpp>
#include <io.hpp>

namespace october {


bbox2f_t
project_line_representations_(const mat4f_t& projection_0, const mat4f_t& projection_1, const std::vector<vec3f_t>& input_0, const std::vector<vec3f_t>& input_1, std::vector<vec2f_t>& output_0, std::vector<vec2f_t>& output_1, vec2f_t& center_0, vec2f_t& center_1) {
    // project lines to 2D space
    output_0.resize(input_0.size());
    output_1.resize(input_1.size());
    std::transform(input_0.begin(), input_0.end(), output_0.begin(), [&] (const vec3f_t& pos) { return (projection_0 * pos.homogeneous()).head(2); });
    std::transform(input_1.begin(), input_1.end(), output_1.begin(), [&] (const vec3f_t& pos) { return (projection_1 * pos.homogeneous()).head(2); });

    // compute 2d bounding boxes
    bbox2f_t bb_a, bb_b;
    for (const vec2f_t& pos : output_0) {
        bb_a.extend(pos);

    }
    for (const vec2f_t& pos : output_1) {
        bb_b.extend(pos);
    }
    center_0 = bb_a.center();
    center_1 = bb_b.center();

    // center lines
    bbox2f_t merged_bbox;
    for (vec2f_t& pos : output_0) {
        pos -= center_0;
        merged_bbox.extend(pos);
    }
    for (vec2f_t& pos : output_1) {
        pos -= center_1;
        merged_bbox.extend(pos);
    }

    return merged_bbox;
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

mat4f_t
align_lines_2d(const mat4f_t& projection_0, const mat4f_t& projection_1, const std::vector<vec3f_t>& lines_0, const std::vector<vec3f_t>& lines_1) {
    std::vector<vec2f_t> lines_2d_0, lines_2d_1;
    vec2f_t center_0, center_1;
    bbox2f_t bbox = project_line_representations_(projection_0, projection_1, lines_0, lines_1, lines_2d_0, lines_2d_1, center_0, center_1);
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
            max_response = response;
        }
    }
    auto final_img = affine_transformed_image(img_0, final_transform, 0.f);
    //write_image(final_img, "/tmp/a.png");
    //write_image(img_1, "/tmp/b.png");

    mat4f_t local_transform = mat4f_t::Identity(), center_mat_0 = mat4f_t::Identity(), center_mat_1 = mat4f_t::Identity();
    local_transform.block<2,2>(0, 0) = final_transform.linear();
    local_transform.block<2,1>(0, 3) = final_transform.translation();
    center_mat_0.block<2,1>(0, 3) = -center_0;
    center_mat_1.block<2,1>(0, 3) =  center_1;

    return projection_1.inverse() * center_mat_1 * local_transform * center_mat_0 * projection_0;
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


} // october
