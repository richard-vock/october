#ifndef _OCTOBER_BOUNDED_PLANE_HPP_
#define _OCTOBER_BOUNDED_PLANE_HPP_

#include "types.hpp"

namespace october {

class bounded_plane {
    public:
        typedef std::shared_ptr<bounded_plane>       ptr_t;
        typedef std::weak_ptr<bounded_plane>         wptr_t;
        typedef std::shared_ptr<const bounded_plane> const_ptr_t;
        typedef std::weak_ptr<const bounded_plane>   const_wptr_t;

    public:
        static ptr_t from_points(const std::vector<vec3f_t>& points);

        virtual ~bounded_plane();

        const mat3f_t& base() const;

        vec3f_t normal() const;

        const vec3f_t& origin() const;

        const std::vector<vec3f_t>& points() const;

        const bbox2f_t& local_bounding_box() const;

        const bbox3f_t& global_bounding_box() const;

        vec2f_t project_2d(const vec3f_t& position) const;

        vec3f_t project_3d(const vec3f_t& position) const;

        std::vector<vec3f_t> area_polygon() const;

        float local_exterior_distance(const_ptr_t other) const;

        std::pair<vec2f_t, vec2f_t> project_2d_line(const vec3f_t& project_dir) const;

        bool is_orthogonal_to(const vec3f_t& dir, float angular_precision = Eigen::NumTraits<float>::epsilon()) const;

        bool is_perpendicular_to(const vec3f_t& dir, float angular_precision = Eigen::NumTraits<float>::epsilon()) const;

        void transform(const mat4f_t& transformation);

    protected:
        bounded_plane(const mat3f_t& base, const vec3f_t& origin, const std::vector<vec3f_t>& points);

    protected:
        mat3f_t               base_;
        vec3f_t               origin_;
        std::vector<vec3f_t>  points_;
        bbox2f_t              local_bbox_;
        bbox3f_t              global_bbox_;
};


} // october

#endif /* _OCTOBER_BOUNDED_PLANE_HPP_ */
