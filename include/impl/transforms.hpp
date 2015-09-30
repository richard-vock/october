#ifndef _OCTOBER_TRANSFORMS_IMPL_HPP_
#define _OCTOBER_TRANSFORMS_IMPL_HPP_


template <int D>
inline Eigen::Matrix<float, D+1, D+1> affine_image_transform(const Eigen::AlignedBox<float, D>& bbox, const Eigen::Matrix<int, D, 1>& img_size, int border) {
    typedef Eigen::Matrix<float,   D,   D>  linear_mat_t;
    typedef Eigen::Matrix<float, D+1, D+1>  affine_mat_t;

    affine_mat_t transform = affine_mat_t::Identity();

    // to unit cube
    transform.template block<D, 1>(0, D) = -bbox.center();

    float bbox_range = sqrtf(2.f) * (bbox.max() - bbox.min()).maxCoeff();
    std::cout << "d: " << (bbox.max() - bbox.min()) << "\n";
    std::cout << "r: " << (bbox.max() - bbox.min()).maxCoeff() << "\n";
    affine_mat_t scale_and_mirror = affine_mat_t::Identity();
    scale_and_mirror.template block<D, D>(0, 0) *= 1.f / bbox_range;
    if (D > 1) scale_and_mirror(1,1) *= -1.f;
    transform = scale_and_mirror * transform;

    Eigen::Matrix<float, D, 1> sizes = img_size.template cast<float>();
    Eigen::Matrix<float, D, 1> borders = Eigen::Matrix<float, D, 1>::Constant(static_cast<float>(border));


    affine_mat_t stretch = affine_mat_t::Identity();
    for (int i = 0; i < D; ++i) {
        stretch(i,i) = sizes[i] - 2.f * borders[i];
    }
    transform = stretch * transform;

    transform.template block<D, 1>(0, D) += 0.5f * sizes;

    return transform;
}


#endif /* _OCTOBER_TRANSFORMS_IMPL_HPP_ */
