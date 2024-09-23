
#include "mess2_algorithm_plugins/low_level/low_level.hpp"

namespace mess2_algorithms
{
    LowLevelSearch::LowLevelSearch() {};

    void LowLevelSearch::fill_low_level_search(const Graph& graph, Actor& actor, const int64_t& index_source, const int64_t& index_target)
    {
        graph_ = graph;

        adjacency_wait_ = generate_adjacency(graph, "wait");
        // (void) adjacency_wait_.print_adjacency(graph);

        adjacency_rotate_ = generate_adjacency(graph, "rotate");
        // (void) adjacency_rotate_.print_adjacency(graph);

        adjacency_translate_ = generate_adjacency(graph, "translate");
        // (void) adjacency_translate_.print_adjacency(graph);

        actor_ = actor;

        index_source_ = index_source;
        index_target_ = index_target;
        heuristic_.fill_heuristic(actor_.get_scores(), graph_, index_target_);

        key_edges_ = generate_edges_key(graph_.get_edges().size());
    }

    pathplan LowLevelSearch::execute_low_level_search(const Constraints& constraints)
    {
        (void) history_.clear_history();
        (void) queue_.clear_queue();

        (void) history_.append_history(0.0, 0.0, index_source_, -1, "wait");
        (void) queue_.append_queue(0.0, 0.0, index_source_, 0);

        bool is_complete = false;
        int64_t counter_update = 0;
        while (!queue_.is_empty())
        {
            const auto curr = queue_.lookup_queue();
            const auto last = history_.lookup_history(curr.index_history);

            const auto size_init = queue_.size_queue();

            std::vector<int64_t> adjacencies;
            if (last.type=="wait") {
                adjacencies = adjacency_wait_.get_adjacencies(curr.index_parent);
            } else if (last.type=="rotate") {
                adjacencies = adjacency_rotate_.get_adjacencies(curr.index_parent);
            } else if (last.type=="translate") {
                adjacencies = adjacency_translate_.get_adjacencies(curr.index_parent);
            }

            auto index_history = history_.size_history();
            for (int64_t iter = 0; iter < static_cast<int64_t>(adjacencies.size()); ++iter)
            {
                const auto index_edge = adjacencies[iter];

                const auto edge = graph_.get_edge(index_edge);
                const auto index_child = edge.get_index_child();

                const auto key_edge = key_edges_[index_edge];
                if (key_edge == 0) {
                    auto [ds, dt] = actor_.get_cost_to_transition(index_edge);

                    const auto dh = heuristic_.lookup_heuristic(index_edge) * dt; // ds premultiplied by dt in func

                    const auto score_next = curr.score + ds + dh;
                    const auto time_next = curr.time + dt;
                    const auto type_next = edge.get_type();

                    bool is_constrained = false;
                    const auto vertex_constraints = constraints.lookup_constraints_at_vertex(index_child);
                    for (const auto& constraint : vertex_constraints) {
                        if (constraint.t_init < time_next && constraint.t_term > time_next) {
                            is_constrained = true;
                        }
                    }
                    
                    if (!is_constrained) {
                        history_.append_history(score_next, time_next, index_child, curr.index_history, type_next);
                        queue_.append_queue(score_next, time_next, index_child, index_history);

                        index_history += 1;
                        if (index_child==index_target_) {
                            is_complete = true;
                            break;
                        }
                    }

                    key_edges_[index_edge] += 1;
                }
            }

            const auto size_term = queue_.size_queue();
            if (size_init != size_term) {
                counter_update = 0;
            } else if (size_init == size_term) {
                counter_update = counter_update + 1;
            }

            if (queue_.size_queue() == 0) {
                
                key_edges_ = reset_edges_key(key_edges_);
                const auto vertex_curr = graph_.get_vertex(curr.index_parent);

                for (int64_t iter = 0; iter < counter_update; ++iter) {
                    const auto index_history = history_.size_history() - (1 + iter);
                    const auto history = history_.lookup_history(index_history);

                    const auto vertex_history = graph_.get_vertex(history.index_parent);

                    if (vertex_curr.get_x() == vertex_history.get_x() && vertex_curr.get_y() == vertex_history.get_y() && history.type == "wait") {
                        const auto index_edge = lookup_index_edge(graph_, history.index_parent, history.index_parent);

                        auto [ds, dt] = actor_.get_cost_to_transition(index_edge);

                        const auto dh = heuristic_.lookup_heuristic(index_edge) * dt; // ds premultiplied by dt in func

                        const auto score_next = history.score + ds + dh;
                        const auto time_next = history.time + dt;

                        history_.append_history(score_next, time_next, history.index_parent, index_history, "wait");
                        queue_.append_queue(score_next, time_next, history.index_parent, history_.size_history() - 1);

                        

                        // for the time being only allow waiting if no edges have traversable edges
                    }
                }
            }

            if (is_complete) {
                break;
            }
        }

        auto path = retrieve_path(history_);

        return path;
    }

    Graph LowLevelSearch::get_graph() {
        return graph_;
    }

} // namespace mess2_algorithm_plugins

