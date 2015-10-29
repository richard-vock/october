#ifndef _OCTOBER_REPRESENTATION_HPP_
#define _OCTOBER_REPRESENTATION_HPP_

#include <vector>

#include "adapter.hpp"

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/none.hpp>
namespace fs = boost::filesystem;


namespace october {


class representation {
    public:
        typedef std::shared_ptr<representation>       ptr_t;
        typedef std::weak_ptr<representation>         wptr_t;
        typedef std::shared_ptr<const representation> const_ptr_t;
        typedef std::weak_ptr<const representation>   const_wptr_t;

    public:
        static ptr_t from_files(const std::vector<fs::path>& file_paths, float area_threshold, float angle_threshold, const std::vector<adapter::ptr_t>& adapters);

        ~representation();

        const std::vector<bounded_plane::ptr_t>& planes() const;

        void transform(const mat4f_t& transformation);

        const std::string& guid() const;

    protected:
        representation(const std::vector<bounded_plane::ptr_t>& planes, const std::string& guid);

#ifdef WITH_CEREAL
    public:
        representation() = default;
        CEREAL_ACCESS

        template <typename Archive>
        void serialize(Archive& ar);
#endif // WITH_CEREAL

    protected:
        std::vector<bounded_plane::ptr_t>  planes_;
        std::string guid_;
};

#include "impl/representation.hpp"


} // october


#endif /* _OCTOBER_REPRESENTATION_HPP_ */
