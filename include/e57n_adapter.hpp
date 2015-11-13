#ifndef _OCTOBER_E57N_ADAPTER_HPP_
#define _OCTOBER_E57N_ADAPTER_HPP_
#if defined(WITH_E57PCL) && defined(WITH_PRIMITIVE_DETECTION)

#include "adapter.hpp"

namespace october {


class e57n_adapter : public adapter {
    public:
        typedef std::shared_ptr<e57n_adapter>       ptr_t;
        typedef std::weak_ptr<e57n_adapter>         wptr_t;
        typedef std::shared_ptr<const e57n_adapter> const_ptr_t;
        typedef std::weak_ptr<const e57n_adapter>   const_wptr_t;

    public:
        e57n_adapter(float epsilon, float bitmap_epsilon, uint32_t min_support, float probability);

        virtual
        ~e57n_adapter();

        bool
        supports_extension(const std::vector<std::string>& extensions);

        std::vector<bounded_plane::ptr_t>
        extract_planes(const std::vector<std::string>& file_paths, float area_threshold, float angle_threshold, std::string& guid) override;

    protected:
        float     epsilon_;
        float     bitmap_epsilon_;
        uint32_t  min_support_;
        float     probability_;
};


} // october

#endif // defined(WITH_E57PCL) && defined(WITH_PRIMITIVE_DETECTION)
#endif /* _OCTOBER_E57N_ADAPTER_HPP_ */
