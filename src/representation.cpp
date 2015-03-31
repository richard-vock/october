#include <representation.hpp>

namespace october {


representation::~representation() {
}

representation::representation(const std::vector<bounded_plane::ptr_t>& planes) : planes_(planes) {
}

const std::vector<bounded_plane::ptr_t>&
representation::planes() const {
    return planes_;
}

#ifdef WITH_CEREAL

representation::representation() {
}

#endif // WITH_CEREAL


} // october
