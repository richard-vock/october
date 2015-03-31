#include <bounded_plane.hpp>
#include <utils.hpp>

namespace october {

bounded_plane::ptr_t bounded_plane::from_points(const std::vector<vec3f_t>& points) {
    mat3f_t base;
    vec3f_t origin;
    pca(points, base, origin);
    return ptr_t(new bounded_plane(base, origin, points));
}

bounded_plane::~bounded_plane() {
}

const mat3f_t&
bounded_plane::base() const {
    return base_;
}

vec3f_t
bounded_plane::normal() const {
    return base_.col(2);
}

const vec3f_t&
bounded_plane::origin() const {
    return origin_;
}

const std::vector<vec3f_t>&
bounded_plane::points() const {
    return points_;
}

const bbox2f_t&
bounded_plane::local_bounding_box() const {
    return local_bbox_;
}

const bbox3f_t&
bounded_plane::global_bounding_box() const {
    return global_bbox_;
}

vec2f_t
bounded_plane::project_2d(const vec3f_t& position) const {
    return project_3d(position).head(2);
}

vec3f_t
bounded_plane::project_3d(const vec3f_t& position) const {
    return base_.transpose() * (position - origin_);
}

std::vector<vec3f_t>
bounded_plane::area_polygon() const {
    vec2f_t p0 = local_bbox_.min(), p1, p2 = local_bbox_.max(), p3;
    p1 << p2[0], p0[1];
    p3 << p0[0], p2[1];

    vec3f_t u = base_.col(0);
    vec3f_t v = base_.col(1);
    return {
        origin_ + p0[0] * u + p0[1] * v,
        origin_ + p1[0] * u + p1[1] * v,
        origin_ + p2[0] * u + p2[1] * v,
        origin_ + p3[0] * u + p3[1] * v
    };
}

float
bounded_plane::local_exterior_distance(const_ptr_t other) const {
    std::vector<vec3f_t> poly_0 = area_polygon();
    std::vector<vec3f_t> poly_1 = other->area_polygon();
    bbox2f_t proj_bbox_0, proj_bbox_1;
    for (const auto& p : poly_1) {
        proj_bbox_0.extend(project_2d(p));
    }
    for (const auto& p : poly_0) {
        proj_bbox_1.extend(other->project_2d(p));
    }

    return std::min(proj_bbox_0.exteriorDistance(local_bbox_), proj_bbox_1.exteriorDistance(other->local_bbox_));
}

std::pair<vec2f_t, vec2f_t>
bounded_plane::project_2d_line(const vec3f_t& project_dir) const {
    std::vector<vec3f_t> poly = area_polygon();
    vec3f_t line_dir = project_dir.cross(base_.col(2)).normalized();
    std::vector<float> lambdas;
    for (const auto& p : poly) {
        lambdas.push_back(line_dir.dot(p - origin_));
    }

    vec2f_t start = (origin_ + (*std::min_element(lambdas.begin(), lambdas.end())) * line_dir).head(2);
    vec2f_t end = (origin_ + (*std::max_element(lambdas.begin(), lambdas.end())) * line_dir).head(2);
    return {start, end};
}

bool
bounded_plane::is_orthogonal_to(const vec3f_t& dir, float angular_precision) const {
    return (1.f - fabs(normal().dot(dir))) < angular_precision;
}

bool
bounded_plane::is_perpendicular_to(const vec3f_t& dir, float angular_precision) const {
    return fabs(normal().dot(dir)) < angular_precision;
}

void
bounded_plane::transform(const mat4f_t& transformation) {
    affine3f_t t;
    t = transformation;
    origin_ = t * origin_;
    base_ = t.linear() * base_;
    local_bbox_ = bbox2f_t();
    global_bbox_ = bbox3f_t();
    for (auto& p : points_) {
        p = t * p;
        global_bbox_.extend(p);
        local_bbox_.extend(project_2d(p));
    }
}

bounded_plane::bounded_plane(const mat3f_t& base, const vec3f_t& origin, const std::vector<vec3f_t>& points) : base_(base), origin_(origin), points_(points) {
    local_bbox_ = bbox2f_t();
    global_bbox_ = bbox3f_t();
    for (const auto& p : points_) {
        global_bbox_.extend(p);
        local_bbox_.extend(project_2d(p));
    }
}

#ifdef WITH_CEREAL
bounded_plane::bounded_plane() {
}
#endif // WITH_CEREAL


} // october
