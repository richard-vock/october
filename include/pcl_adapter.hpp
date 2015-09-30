#ifndef _OCTOBER_PCL_ADAPTER_HPP_
#define _OCTOBER_PCL_ADAPTER_HPP_
#if defined(WITH_PCL) && defined(WITH_PRIMITIVE_DETECTION)

#include "adapter.hpp"

namespace october {


class pcl_adapter : public adapter {
    public:
        typedef std::shared_ptr<pcl_adapter>       ptr_t;
        typedef std::weak_ptr<pcl_adapter>         wptr_t;
        typedef std::shared_ptr<const pcl_adapter> const_ptr_t;
        typedef std::weak_ptr<const pcl_adapter>   const_wptr_t;

    public:
        pcl_adapter(float epsilon, float bitmap_epsilon, uint32_t min_support, float probability);

        virtual
        ~pcl_adapter();

        bool
        supports_extension(const std::vector<std::string>& extensions);

        std::vector<bounded_plane::ptr_t>
        extract_planes(const std::vector<std::string>& file_paths, float area_threshold, float angle_threshold);

    protected:
        float     epsilon_;
        float     bitmap_epsilon_;
        uint32_t  min_support_;
        float     probability_;
};


} // october

#endif // defined(WITH_PCL) && defined(WITH_PRIMITIVE_DETECTION)
#endif /* _OCTOBER_PCL_ADAPTER_HPP_ */
