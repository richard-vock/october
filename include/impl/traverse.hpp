#ifndef _OCTOBER_TRAVERSE_IMPL_HPP_
#define _OCTOBER_TRAVERSE_IMPL_HPP_


template <typename Entity, typename FNeigh, typename FPred, typename Visitor>
inline void
traverse(Entity start, FNeigh&& get_neighbors, FPred&& valid_neighbor, Visitor&& visitor, bool depth_first) {
    std::deque<Entity> todo = { start };
    std::set<Entity> considered = { start };

    while (!todo.empty()) {
        Entity current = todo.front();
        todo.pop_front();

        visitor(current);

        for (auto neighbor : get_neighbors(current)) {
            if (considered.find(neighbor) != considered.end() || !valid_neighbor(neighbor)) continue;
            considered.insert(neighbor);
            if (depth_first) {
                todo.push_front(neighbor);
            } else {
                todo.push_back(neighbor);
            }
        }
    }
}

template <typename Entity, typename FNeigh, typename FCompPred, typename FPred>
inline connected_components_t<Entity>
connected_components(const std::vector<Entity>& entities, FNeigh&& get_neighbors, FCompPred&& in_component, FPred&& valid_start) {
    std::set<Entity> todo(entities.begin(), entities.end()), visited;

    connected_components_t<Entity> components;
    while (visited.size() < todo.size()) {
        // find first unvisited entity
        Entity start = *std::find_if(todo.begin(), todo.end(), [&] (Entity entity) { return visited.find(entity) == visited.end(); });
        if (!valid_start(start)) {
            visited.insert(start);
            continue;
        }

        connected_component_t<Entity> component;
        traverse(
            start,
            std::forward<FNeigh>(get_neighbors),
            [&] (Entity entity) {
                return in_component(entity, start);
            },
            [&] (Entity entity) {
                component.push_back(entity);
                visited.insert(entity);
            },
            false
        );
        components.push_back(component);
    }

    return components;
}


#endif /* _OCTOBER_TRAVERSE_IMPL_HPP_ */
