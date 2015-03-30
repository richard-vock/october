#ifndef GREEDY_CLUSTER_HPP_
#define GREEDY_CLUSTER_HPP_

#include <vector>
#include <functional>

#include "impl/greedy_cluster_traits.hpp"

namespace october {

template <typename Entities, typename CreateCluster, typename IsInCluster, typename InsertIntoCluster>
std::vector<derived_cluster_t<CreateCluster>>
greedy_cluster(Entities&& entities, IsInCluster&& inclusion_predicate, InsertIntoCluster&& insert, CreateCluster&& create);


#include "impl/greedy_cluster.hpp"


} // october

#endif /* GREEDY_CLUSTER_HPP_ */
