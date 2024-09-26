
#include "mess2_algorithm_plugins/low_level_search.hpp"

namespace mess2_algorithms
{
    LowLevelSearch::LowLevelSearch() {};

    low_level_search_output LowLevelSearch::execute_low_level_search(const Graph& graph, const Actor& actor, const ConstraintsVertices& constraints, const double& timeout)
    {
        map_.resize(graph.n_edges);
        std::fill(map_.begin(), map_.end(), 0);

        (void) history_.clear_history();
        (void) queue_.clear_queue();

        (void) history_.append_history(0.0, 0.0, actor.index_source_, -1, "wait");
        (void) queue_.append_queue(0.0, 0.0, actor.index_source_, 0);

        const auto t_init = std::chrono::steady_clock::now();
        bool is_complete = false;
        int64_t is_updated = 0;
        while (queue_.size_queue() > 0)
        {
            bool is_timeout = (std::chrono::duration<double>(std::chrono::steady_clock::now() - t_init).count() > timeout);
            if (is_timeout) {
                throw std::runtime_error("low level search timed out");
                break;
            }

            const auto curr = queue_.lookup_queue();
            const auto last = history_.lookup_history(curr.index_history);
            const auto size_curr = queue_.size_queue();

            const auto adjacencies = graph.lookup_adjacencies(curr.index_parent, last.type);

            auto index_history = history_.size_history();
            for (int64_t iter = 0; iter < static_cast<int64_t>(adjacencies.size()); ++iter) {
                const auto index_edge = adjacencies[iter];
                if (map_[index_edge] == 0) {
                    const auto dc = actor.lookup_cost(index_edge);
                    const auto dt = actor.lookup_time(index_edge);
                    const auto dh = actor.lookup_heuristic(index_edge);

                    const auto edge = graph.lookup_edge(index_edge);
                    const auto score_next = curr.score + (dc + dh) * dt;
                    const auto time_next = curr.time + dt;
                    const auto type_next = edge.type;

                    bool is_constrained = false;
                    const auto constraints_child = constraints.lookup_vertex(edge.index_child);
                    
                    for (const auto& constraint_child : constraints_child) {
                        if (constraint_child.t_init <= time_next && constraint_child.t_term >= time_next) {
                            is_constrained = true;
                        }
                    }

                    if (!is_constrained) {
                        history_.append_history(score_next, time_next, edge.index_child, curr.index_history, type_next);
                        queue_.append_queue(score_next, time_next, edge.index_child, index_history);
                        index_history += 1;

                        if (edge.index_child == actor.index_target_) {
                            is_complete = true;
                            break;
                        }
                    }

                    map_[index_edge] += 1;
                }
            }

            const auto size_term = queue_.size_queue();
            if (size_curr != size_term) {
                is_updated = 0;
            } else if (size_curr == size_term) {
                is_updated += 1;
            }

            if (queue_.size_queue() == 0) {
                std::fill(map_.begin(), map_.end(), 0);
                const auto vertex_curr = graph.lookup_vertex(curr.index_parent);

                for (int64_t iter = 0; iter < is_updated; ++iter) {
                    const auto index_history = history_.size_history() - (1 + iter);
                    const auto history = history_.lookup_history(index_history);
                    const auto vertex_history = graph.lookup_vertex(history.index_parent);

                    // assume if you are currently at a vertex, you may continue to wait at said vertex
                    if (vertex_curr.x == vertex_history.x && vertex_curr.y == vertex_history.y && history.type == "wait") {
                        // get edge index and append queue with wait
                        const auto index_edge = graph.lookup_index_edge(history.index_parent, history.index_parent);

                        const auto dc = actor.lookup_cost(index_edge);
                        const auto dt = actor.lookup_time(index_edge);
                        const auto dh = actor.lookup_heuristic(index_edge);

                        const auto edge = graph.lookup_edge(index_edge);
                        const auto score_next = curr.score + (dc + dh) * dt;
                        const auto time_next = curr.time + dt;
                        const auto type_next = edge.type;

                        history_.append_history(score_next, time_next, edge.index_child, curr.index_history, type_next);
                        queue_.append_queue(score_next, time_next, edge.index_child, index_history);
                    }
                }
            }

            if (is_complete) {
                break;
            }
        }

        low_level_search_output output;
        const auto index_history = history_.size_history() - 1;
        output.score = history_.unpack_score(index_history);
        output.path = history_.unpack_path(index_history);
        return output;
    }

    void LowLevelSearch::info()
    {
        throw std::runtime_error("low level search info not implemented");
    }

    void info_low_level_search_output(const low_level_search_output& output, const Graph& graph, Actor& actor)
    {
        const auto vertex_source = graph.lookup_vertex(actor.index_source_);
        const auto vertex_target = graph.lookup_vertex(actor.index_target_);
        std::cout << "goal : ";
        std::cout << "\t" << "(" << vertex_source.x << ", " << vertex_source.y << ", " << vertex_source.theta << ") to (" << vertex_target.x << ", " << vertex_target.y << ", " << vertex_target.theta << ")" << std::endl;
        std::cout << "score :";
        std::cout << "\t" << output.score << std::endl;
        std::cout << "path :";
        for (const auto& path : output.path) {
            const auto index_vertex = path.second;
            const auto vertex = graph.lookup_vertex(index_vertex);
            std::cout << "\t(" << vertex.x << ", " << vertex.y << ", " << vertex.theta << ")" << std::endl;
        }
    }

} // namespace mess2_algorithms
