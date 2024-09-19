
#include "mess2_algorithm_plugins/low_level/_low_level.hpp"

namespace mess2_algorithms
{
    LowLevelSearch::LowLevelSearch(const Graph& graph) {
        graph_ = graph;
        adjacency_wait_ = generate_adjacency(graph, "wait");
        adjacency_rotate_ = generate_adjacency(graph, "rotate");
        adjacency_translate_ = generate_adjacency(graph, "translate");
    }

    void LowLevelSearch::execute_low_level_search(const int64_t& index_source, const int64_t& index_target)
    {
        (void) history_.history_clear();
        (void) queue_.queue_clear();

        (void) history_.history_append(0.0, 0.0, index_source, -1, "wait");
        (void) queue_.queue_append(0.0, 0.0, index_source, 0);

        bool is_complete = false;
        while (!queue_.is_empty())
        {
            const auto curr = queue_.queue_lookup();
            const auto last = history_.history_lookup(curr.index_history);

            std::vector<int64_t> adjacencies;
            if (last.type=="wait") {
                const auto adjacencies = adjacency_wait_.get_adjacencies(curr.index_parent);
            } else if (last.type=="rotate") {
                const auto adjacencies = adjacency_rotate_.get_adjacencies(curr.index_parent);
            } else if (last.type=="translate") {
                const auto adjacencies = adjacency_translate_.get_adjacencies(curr.index_parent);
            }

            for (int64_t iter =0; iter < static_cast<int64_t>(adjacencies.size()); ++iter)
            {
                const auto index_edge = adjacencies[iter];
                const auto edge = graph_.get_edge_(index_edge);

                // auto cost = get_cost_to_transition();

                // const auto index_child = edge.get_index_child_();
                // const auto type = edge.get_type_();
            }
        }
    }

} // namespace mess2_algorithm_plugins

