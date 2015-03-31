#ifndef _OCTOBER_REPRESENTATION_HPP_
#define _OCTOBER_REPRESENTATION_HPP_

#include "bounded_plane.hpp"

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/none.hpp>
namespace fs = boost::filesystem;


namespace october {


class representation {
    CEREAL_ACCESS

    public:
        typedef std::shared_ptr<representation>       ptr_t;
        typedef std::weak_ptr<representation>         wptr_t;
        typedef std::shared_ptr<const representation> const_ptr_t;
        typedef std::weak_ptr<const representation>   const_wptr_t;

    public:
        template <typename... Adapters>
        static ptr_t from_file(const fs::path& file_path, float area_threshold, float angle_threshold);

        virtual ~representation();

        const std::vector<bounded_plane::ptr_t>& planes() const;

    protected:
        representation(const std::vector<bounded_plane::ptr_t>& planes);

#ifdef WITH_CEREAL
    private:
        representation();

        template <typename Archive>
        void serialize(Archive& ar);
#endif // WITH_CEREAL

    protected:
        std::vector<bounded_plane::ptr_t>  planes_;
};

#include "impl/representation.hpp"


} // october

#endif /* _OCTOBER_REPRESENTATION_HPP_ */
