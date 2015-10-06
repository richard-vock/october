#include <bounded_plane.hpp>
#include <utils.hpp>
#include <greedy_cluster.hpp>

namespace october {

bounded_plane::ptr_t bounded_plane::from_points(const std::vector<vec3f_t>& points, const vec3f_t& normal) {
    mat3f_t base;
    vec3f_t origin;
    pca(points, base, origin);
    // If an (approximate) normal is given and the computed
    // normal points in the opposite direction, flip the basis.
    if (!normal.isZero()) {
        if (base.col(2).dot(normal) < 0.f) {
            base *= -1.f;
        }
    }
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
    vec3f_t line_dir = project_dir.cross(base_.col(2)).normalized();
    std::vector<float> lambdas;
    for (const auto& p : points_) {
        lambdas.push_back(line_dir.dot(p - origin_));
    }

    vec2f_t start = (origin_ + (*std::min_element(lambdas.begin(), lambdas.end())) * line_dir).head(2);
    vec2f_t end = (origin_ + (*std::max_element(lambdas.begin(), lambdas.end())) * line_dir).head(2);
    return {start, end};
}

std::pair<vec2f_t, vec2f_t> bounded_plane::project_2d_line(const mat4f_t& pre_transform, const vec3f_t& project_dir) const {
    vec3f_t nrm = base_.col(2);
    nrm = (pre_transform * nrm.homogeneous()).head(3);
    vec3f_t line_dir = project_dir.cross(nrm).normalized();
    std::vector<float> lambdas;
    vec3f_t origin = (pre_transform * origin_.homogeneous()).head(3);
    for (const auto& p : points_) {
        vec3f_t point = (pre_transform * p.homogeneous()).head(3);
        lambdas.push_back(line_dir.dot(point - origin));
    }

    vec2f_t start = (origin + (*std::min_element(lambdas.begin(), lambdas.end())) * line_dir).head(2);
    vec2f_t end = (origin + (*std::max_element(lambdas.begin(), lambdas.end())) * line_dir).head(2);
    return {start, end};
}

bool
bounded_plane::is_orthogonal_to(const vec3f_t& dir, float angular_precision) const {
    return (1.f - fabs(normal().dot(dir))) < angular_precision;
}

bool
bounded_plane::is_orthogonal_to(const mat4f_t& pre_transform, const vec3f_t& dir, float angular_precision) const {
    vec3f_t nrm = (pre_transform * normal().homogeneous()).head(3);
    return (1.f - fabs(nrm.dot(dir))) < angular_precision;
}

bool
bounded_plane::is_parallel_to(const vec3f_t& dir, float angular_precision) const {
    return fabs(normal().dot(dir)) < angular_precision;
}

bool
bounded_plane::is_parallel_to(const mat4f_t& pre_transform, const vec3f_t& dir, float angular_precision) const {
    vec3f_t nrm = (pre_transform * normal().homogeneous()).head(3);
    return fabs(nrm.dot(dir)) < angular_precision;
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

mat3f_t primary_normal_directions(const std::vector<bounded_plane::ptr_t>& planes, float cluster_cos_threshold, float vertical_cos_threshold) {
    typedef std::vector<vec3f_t> cluster_t;
    std::vector<vec3f_t> directions;
    for (const auto& plane : planes) {
        if (fabs(plane->normal()[2]) < vertical_cos_threshold) {
            directions.push_back(plane->normal());
        }
    }
    //std::transform(planes.begin(), planes.end(), directions.begin(), [&] (bounded_plane::ptr_t p) { return p->normal(); });
    
    std::vector<cluster_t> clusters = greedy_cluster(
        directions,
        // check if dir belongs to cluster
        [&] (const vec3f_t& dir, const cluster_t& cluster) {
            bool included = true;
            for (const vec3f_t& d : cluster) {
                if (1.f - fabs(d.dot(dir)) > cluster_cos_threshold) {
                    included = false;
                    break;
                }
            }
            return included;
        },
        // add direction to cluster
        [&] (const vec3f_t& dir, cluster_t& cluster) {
            return cluster.push_back(dir);
        },
        // create singleton cluster from direction
        [&] (const vec3f_t& dir) {
            return cluster_t(1, dir);
        }
    );
    std::sort(clusters.begin(), clusters.end(), [&] (const cluster_t& a, const cluster_t& b) { return b.size() < a.size(); });
    mat3f_t primary_directions = mat3f_t::Zero();
    for (uint32_t i = 0; i < 2; ++i) {
        vec3f_t delta;
        uint32_t n = 0;
        for (const vec3f_t& dir : clusters[i]) {
            vec3f_t flipped_dir = dir;
            if (dir.dot(vec3f_t::Ones()) < 0.f) flipped_dir *= -1.f;
            ++n;
            delta = flipped_dir - primary_directions.col(i);
            primary_directions.col(i) += delta / static_cast<float>(n);
        }
        primary_directions.col(i).normalize();
    }

    // gram-schmidt
    primary_directions.col(1) -= primary_directions.col(1).dot(primary_directions.col(0)) * primary_directions.col(0);
    primary_directions.col(1).normalize();
    primary_directions.col(2) = primary_directions.col(0).cross(primary_directions.col(1)).normalized();

    return primary_directions;
}


} // october
