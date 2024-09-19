#ifndef MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HPP
#define MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HPP

#include "mess2_algorithm_plugins/common.hpp"

#include "mess2_algorithm_plugins/graph/edge.hpp"
#include "mess2_algorithm_plugins/graph/vertex.hpp"
#include "mess2_algorithm_plugins/graph/_graph.hpp"

#include "mess2_algorithm_plugins/low_level/adjacency.hpp"
#include "mess2_algorithm_plugins/low_level/history.hpp"
#include "mess2_algorithm_plugins/low_level/queue.hpp"

namespace mess2_algorithms
{
    class LowLevelSearch
    {
    public:
        LowLevelSearch(const Graph& graph);

        void execute_low_level_search(const int64_t& index_source, const int64_t& index_target);

    private:
        Graph graph_;
        Adjacency adjacency_wait_;
        Adjacency adjacency_rotate_;
        Adjacency adjacency_translate_;
        LowLevelHistory history_;
        LowLevelQueue queue_;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HPP
