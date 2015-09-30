#ifndef _OCTOBER_BOUNDED_PLANE_IMPL_HPP_
#define _OCTOBER_BOUNDED_PLANE_IMPL_HPP_

#ifdef WITH_CEREAL

template <typename Archive>
inline void
bounded_plane::serialize(Archive& ar) {
    ar(cereal::make_nvp("base", base_));
    ar(cereal::make_nvp("origin", origin_));
    ar(cereal::make_nvp("points", points_));
    ar(cereal::make_nvp("local_bbox", local_bbox_));
    ar(cereal::make_nvp("global_bbox", global_bbox_));
}

#endif // WITH_CEREAL

#endif /* _OCTOBER_BOUNDED_PLANE_IMPL_HPP_ */
