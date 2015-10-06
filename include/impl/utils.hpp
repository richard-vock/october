#ifndef _OCTOBER_UTILS_IMPL_HPP_
#define _OCTOBER_UTILS_IMPL_HPP_


template <typename T, int Rows, int Columns, int Options>
Eigen::Matrix<T, Rows, Columns, Options> compute_centroid(const std::vector<Eigen::Matrix<T, Rows, Columns, Options>>& values) {
    typedef Eigen::Matrix<T, Rows, Columns, Options> value_t;

    uint32_t n = 0;
    value_t centroid = value_t::Constant(0), delta;

    for (const auto& value : values) {
        ++n;
        delta = value - centroid;
        centroid += delta / static_cast<float>(n);
    }

    return centroid;
}


#endif /* _OCTOBER_UTILS_IMPL_HPP_ */
