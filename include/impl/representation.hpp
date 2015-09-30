#ifndef _OCTOBER_REPRESENTATION_IMPL_HPP_
#define _OCTOBER_REPRESENTATION_IMPL_HPP_

#ifdef WITH_CEREAL

template <typename Archive>
inline void
representation::serialize(Archive& ar) {
    ar(cereal::make_nvp("planes", planes_));
}

#endif // WITH_CEREAL


#endif /* _OCTOBER_REPRESENTATION_IMPL_HPP_ */
