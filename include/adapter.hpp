#ifndef _OCTOBER_ADAPTER_HPP_
#define _OCTOBER_ADAPTER_HPP_

#include "bounded_plane.hpp"

namespace october {


class adapter {
    public:
        typedef std::shared_ptr<adapter>       ptr_t;
        typedef std::weak_ptr<adapter>         wptr_t;
        typedef std::shared_ptr<const adapter> const_ptr_t;
        typedef std::weak_ptr<const adapter>   const_wptr_t;

    public:
        adapter() {}

        virtual
        ~adapter() {}

        virtual bool
        supports_extension(const std::vector<std::string>& extensions) = 0;

        virtual std::vector<bounded_plane::ptr_t>
        extract_planes(const std::vector<std::string>& file_paths, float area_threshold, float angle_threshold, std::string& guid) = 0;
};


} // october

#endif /* _OCTOBER_ADAPTER_HPP_ */
