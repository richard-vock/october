#ifndef _OCTOBER_REPRESENTATION_IMPL_HPP_
#define _OCTOBER_REPRESENTATION_IMPL_HPP_

template <typename... Adapters>
struct representation_loader;

template <typename Adapter, typename... Adapters>
struct representation_loader<Adapter, Adapters...> {
    static boost::optional<std::vector<bounded_plane::ptr_t>>
    load(const fs::path& file_path, float area_threshold, float angle_threshold) {
        if (!Adapter::supports_extension(file_path.extension().string())) return representation_loader<Adapters...>::load(file_path, area_threshold, angle_threshold);
        std::vector<bounded_plane::ptr_t> planes = Adapter::extract_planes(file_path.string(), area_threshold, angle_threshold);
        return planes;
    }
};

template <>
struct representation_loader<> {
    static boost::optional<std::vector<bounded_plane::ptr_t>>
    load(const fs::path& file_path, float area_threshold, float angle_threshold) {
        return boost::none;
    }
};

template <typename... Adapters>
inline representation::ptr_t
representation::from_file(const fs::path& file_path, float area_threshold, float angle_threshold) {
    auto planes = representation_loader<Adapters...>::load(file_path, area_threshold, angle_threshold);
    if (!planes) {
        throw std::runtime_error("representation::from_file(const fs::path&): No suitable adapter found.");
    }
    return ptr_t(new representation(*planes));
}

#ifdef WITH_CEREAL

template <typename Archive>
inline void
representation::serialize(Archive& ar) {
    cereal::make_nvp("planes", planes_);
}

#endif // WITH_CEREAL


#endif /* _OCTOBER_REPRESENTATION_IMPL_HPP_ */
