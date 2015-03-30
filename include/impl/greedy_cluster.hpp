#ifndef GREEDY_CLUSTER_IMPL_HPP_
#define GREEDY_CLUSTER_IMPL_HPP_


template <typename Entities, typename CreateCluster, typename IsInCluster, typename InsertIntoCluster>
std::vector<derived_cluster_t<CreateCluster>>
greedy_cluster(Entities&& entities, IsInCluster&& inclusion_predicate, InsertIntoCluster&& insert, CreateCluster&& create) {
    typedef derived_cluster_t<CreateCluster> cluster_t;
    typedef derived_entity_t<CreateCluster>  entity_t;

	std::vector<cluster_t> result;
	for (const entity_t& entity : entities) {
		// check existing clusters for "near-enough" centers
		bool foundCluster = false;
		for (unsigned int c=0; c<result.size(); ++c) {
			if (!inclusion_predicate(entity, result[c])) continue;
			// we found a suitable cluster - update center
			insert(entity, result[c]);
			foundCluster = true;
			break;
		}
		if (foundCluster) continue;
		// no cluster found, create new one
		result.push_back(create(entity));
	}
	return result;
}


#endif /* GREEDY_CLUSTER_IMPL_HPP_ */
