#ifndef _OCTOBER_TRAVERSE_HPP_
#define _OCTOBER_TRAVERSE_HPP_

#include <deque>
#include <vector>
#include <algorithm>

namespace october {


template <typename Entity, typename FNeigh, typename FPred, typename Visitor>
void
traverse(const Entity start, FNeigh&& get_neighbors, FPred&& valid_neighbor, Visitor&& visitor, bool depth_first = true);

template <typename Entity>
using connected_component_t = std::vector<Entity>;

template <typename Entity>
using connected_components_t = std::vector<connected_component_t<Entity>>;

template <typename Entity, typename FNeigh, typename FCompPred, typename FPred = std::function<bool (Entity)>>
connected_components_t<Entity>
connected_components(const std::vector<Entity>& entities, FNeigh&& get_neighbors, FCompPred&& in_component, FPred&& valid_start = [] (Entity) -> bool { return true; });


#include "impl/traverse.hpp"


} // october

#endif /* _OCTOBER_TRAVERSE_HPP_ */
