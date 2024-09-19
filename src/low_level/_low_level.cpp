
#include "mess2_algorithm_plugins/low_level/_low_level.hpp"

namespace mess2_algorithms
{
    LowLevelSearch::LowLevelSearch(const Graph& graph)
    {
        graph_ = graph;
        adjacency_ = generate_adjacency(graph);
    }

    void LowLevelSearch::reset_history() {
        (void) history_.history_clear();
    }

    void LowLevelSearch::reset_queue() {
        (void) queue_.queue_clear();
    }


} // namespace mess2_algorithm_plugins

