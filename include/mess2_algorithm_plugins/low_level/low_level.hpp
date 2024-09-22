#ifndef MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HPP
#define MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/graph.hpp"
#include "mess2_algorithm_plugins/low_level/adjacency.hpp"
#include "mess2_algorithm_plugins/low_level/heuristic.hpp"
#include "mess2_algorithm_plugins/low_level/history.hpp"
#include "mess2_algorithm_plugins/low_level/queue.hpp"
#include "mess2_algorithm_plugins/actor.hpp"
#include "mess2_algorithm_plugins/path.hpp"

namespace mess2_algorithms
{
    class LowLevelSearch
    {
    public:
        LowLevelSearch();

        void fill_low_level_search(const Graph& graph, Actor& actor, const int64_t& index_source, const int64_t& index_target);
        pathplan execute_low_level_search();

    private:
        Graph graph_;
        Adjacency adjacency_wait_;
        Adjacency adjacency_rotate_;
        Adjacency adjacency_translate_;
        LowLevelHistory history_;
        LowLevelQueue queue_;
        Actor actor_;
        
        int64_t index_source_;
        int64_t index_target_;
        Heuristic heuristic_;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HPP
