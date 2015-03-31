#ifndef _OCTOBER_IFCOPENSHELL_ADAPTER_HPP_
#define _OCTOBER_IFCOPENSHELL_ADAPTER_HPP_
#if defined(WITH_IFCOPENSHELL) && defined(WITH_OPENMESH)

#include "bounded_plane.hpp"

namespace october {

struct ifcopenshell_adapter {
    static bool supports_extension(const std::string& extension);

    static std::vector<bounded_plane::ptr_t> extract_planes(const std::string& file_path, float area_threshold, float angle_threshold);
};

} // october

#endif // defined(WITH_IFCOPENSHELL) && defined(WITH_OPENMESH)
#endif /* _OCTOBER_IFCOPENSHELL_ADAPTER_HPP_ */
