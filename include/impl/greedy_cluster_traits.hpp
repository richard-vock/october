#ifndef GREEDY_CLUSTER_TRAITS_HPP_
#define GREEDY_CLUSTER_TRAITS_HPP_

#include <functional>

namespace october {


template <typename F, typename Ret, typename T, typename... Ts>
T
first_argument_helper_(Ret (F::*)(T, Ts...));

template <typename F, typename Ret, typename T, typename... Ts>
T
first_argument_helper_(Ret (F::*)(T, Ts...) const);

template <typename F>
struct first_argument_ {
    typedef decltype(first_argument_helper_(&F::operator())) type;
};

template <typename Sig>
struct first_argument_<std::function<Sig>> {
    typedef typename std::function<Sig>::first_argument_type type;
};


template <typename CreateCluster>
struct cluster_traits {
    typedef typename std::decay<typename first_argument_<CreateCluster>::type>::type  entity_t;
    typedef typename std::result_of<CreateCluster(const entity_t&)>::type    cluster_t;
};

template <typename CreateCluster>
using derived_entity_t = typename cluster_traits<CreateCluster>::entity_t;

template <typename CreateCluster>
using derived_cluster_t = typename cluster_traits<CreateCluster>::cluster_t;


} // october

#endif /* GREEDY_CLUSTER_TRAITS_HPP_ */
