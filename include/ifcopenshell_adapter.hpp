#ifndef _OCTOBER_IFCOPENSHELL_ADAPTER_HPP_
#define _OCTOBER_IFCOPENSHELL_ADAPTER_HPP_
#if defined(WITH_IFCOPENSHELL) && defined(WITH_OPENMESH)

#include "adapter.hpp"

namespace october {

class ifcopenshell_adapter : public adapter {
    public:
        typedef std::shared_ptr<ifcopenshell_adapter>       ptr_t;
        typedef std::weak_ptr<ifcopenshell_adapter>         wptr_t;
        typedef std::shared_ptr<const ifcopenshell_adapter> const_ptr_t;
        typedef std::weak_ptr<const ifcopenshell_adapter>   const_wptr_t;

    public:
        ifcopenshell_adapter();

        virtual
        ~ifcopenshell_adapter();

        bool
        supports_extension(const std::vector<std::string>& extensions);

        std::vector<bounded_plane::ptr_t>
        extract_planes(const std::vector<std::string>& file_paths, float area_threshold, float angle_threshold);
};


} // october

#endif // defined(WITH_IFCOPENSHELL) && defined(WITH_OPENMESH)
#endif /* _OCTOBER_IFCOPENSHELL_ADAPTER_HPP_ */
