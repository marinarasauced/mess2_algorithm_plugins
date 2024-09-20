
#include "mess2_algorithm_plugins/low_level/_low_level.hpp"

namespace mess2_algorithms
{
    LowLevelSearch::LowLevelSearch(const Graph& graph, Actor& actor, const std::vector<double>& threat, const int64_t& index_source, const int64_t& index_target) {
        graph_ = graph;
        adjacency_wait_ = generate_adjacency(graph, "wait");
        adjacency_rotate_ = generate_adjacency(graph, "rotate");
        adjacency_translate_ = generate_adjacency(graph, "translate");

        actor_ = actor; // actor.define_actor outside low level search and fill_actor inside low level search
        actor_.fill_actor(graph, threat);

        index_source_ = index_source;
        index_target_ = index_target;
        heuristic_.heuristic_fill(actor_.get_scores(), graph_, index_target_);
    }

    void LowLevelSearch::execute_low_level_search()
    {
        (void) history_.history_clear();
        (void) queue_.queue_clear();

        (void) history_.history_append(0.0, 0.0, index_source_, -1, "wait");
        (void) queue_.queue_append(0.0, 0.0, index_source_, 0);

        bool is_complete = false;
        while (!queue_.is_empty())
        {
            const auto curr = queue_.queue_lookup();
            const auto last = history_.history_lookup(curr.index_history);

            std::vector<int64_t> adjacencies;
            if (last.type=="wait") {
                adjacencies = adjacency_wait_.get_adjacencies(curr.index_parent);
            } else if (last.type=="rotate") {
                adjacencies = adjacency_rotate_.get_adjacencies(curr.index_parent);
            } else if (last.type=="translate") {
                adjacencies = adjacency_translate_.get_adjacencies(curr.index_parent);
            }

            auto index_history = history_.history_size();
            for (int64_t iter = 0; iter < static_cast<int64_t>(adjacencies.size()); ++iter)
            {
                const auto index_edge = adjacencies[iter];
                const auto edge = graph_.get_edge(index_edge);
                auto [ds, dt] = actor_.get_cost_to_transition(index_edge);

                const auto dh = heuristic_.heuristic_lookup(index_edge);

                const auto score_next = curr.score + ds + dh;
                const auto time_next = curr.time + dt;
                const auto index_parent_next = edge.get_index_child();
                const auto type_next = edge.get_type();
                
                history_.history_append(score_next, time_next, index_parent_next, curr.index_history, type_next);
                queue_.queue_append(score_next, time_next, index_parent_next, index_history);

                index_history += 1;
                if (index_parent_next==index_target_) {
                    is_complete = true;
                    break;
                }
            }

            if (is_complete) {
                break;
            }
        }
    }

} // namespace mess2_algorithm_plugins

