#ifndef _OCTOBER_IFCMESH_ADAPTER_HPP_
#define _OCTOBER_IFCMESH_ADAPTER_HPP_
#ifdef WITH_OPENMESH

#include "adapter.hpp"

namespace october {

class ifcmesh_adapter : public adapter {
    public:
        typedef std::shared_ptr<ifcmesh_adapter>       ptr_t;
        typedef std::weak_ptr<ifcmesh_adapter>         wptr_t;
        typedef std::shared_ptr<const ifcmesh_adapter> const_ptr_t;
        typedef std::weak_ptr<const ifcmesh_adapter>   const_wptr_t;

    public:
        ifcmesh_adapter();

        virtual
        ~ifcmesh_adapter();

        bool
        supports_extension(const std::vector<std::string>& extensions);

        std::vector<bounded_plane::ptr_t>
        extract_planes(const std::vector<std::string>& file_paths, float area_threshold, float angle_threshold);
};


} // october


#endif // WITH_OPENMESH
#endif /* _OCTOBER_IFCMESH_ADAPTER_HPP_ */
