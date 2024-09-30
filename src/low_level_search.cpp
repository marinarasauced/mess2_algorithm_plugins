
#include "mess2_algorithm_plugins/low_level_search.hpp"

namespace mess2_algorithms
{
    LowLevelSearch::LowLevelSearch() {};

    low_level_search_output LowLevelSearch::execute_low_level_search(const Graph& graph, const Actor& actor, const ConstraintsVertices& ct, const int64_t& n_iters)
    {
        auto t_init = std::chrono::steady_clock::now();

        map_.clear();
        map_.resize(graph.n_edges);
        std::fill(map_.begin(), map_.end(), 0);

        (void) history_.clear_history();
        (void) queue_.clear_queue();

        (void) history_.append_history(0.0, 0.0, actor.index_source_, -1, "wait");
        (void) queue_.append_queue(0.0, 0.0, actor.index_source_, 0, 1);

        double t_expand = 0.0;
        for (int64_t iter = 0; iter < graph.n_vertices; ++iter) {
            const auto constraints = ct.lookup_vertex(iter);
            for (const auto& constraint : constraints) {
                if (constraint.t_term > t_expand) {
                    t_expand = constraint.t_term;
                }
            }
        }
        t_expand = 100.0;

        int64_t index_complete = -1;
        int64_t n_iter = 0;
        while (queue_.size_queue() > 0)
        {
            bool is_timedout = (n_iter > n_iters);
            if (is_timedout) {
                throw std::runtime_error("low level search timed out by exceeding max number of iterations");
            }
            n_iter += 1;

            const auto curr = queue_.lookup_queue();
            const auto last = history_.lookup_history(curr.index_history);

            if (curr.index_parent == actor.index_target_ && curr.time >= t_expand) {
                index_complete = curr.index_history;
                break;
            }

            const auto adjacencies = graph.lookup_adjacencies(curr.index_parent, last.type);
            auto index_history = history_.size_history();
            for (const auto& index_edge : adjacencies) {
                const auto edge = graph.lookup_edge(index_edge);
                const auto constraints = ct.lookup_vertex(edge.index_child);

                const auto dt = actor.lookup_time(index_edge);
                const auto time_next = curr.time + dt;

                bool is_constrained = false;
                bool is_other_at_goal = false;
                for (const auto& constraint : constraints) {
                    if (time_next >= constraint.t_init && time_next <= constraint.t_term) {
                        is_constrained = true;
                        if (constraint.t_term == std::numeric_limits<double>::max()) {
                            is_other_at_goal = true;
                        }
                    }
                }
                bool is_visited = map_[index_edge] >= curr.n_visits;

                if (is_constrained) {

                    const auto index_vertex = edge.index_parent;
                    const auto index_last = graph.lookup_index_edge(index_vertex, index_vertex);
                    const auto n_visits_next = curr.n_visits + 1;

                    const auto dt = actor.lookup_time(index_last);
                    const auto time_next = curr.time + dt;

                    const auto dc = actor.lookup_cost(index_last);
                    const auto dh = actor.lookup_heuristic(index_last);
                    const auto score_next = curr.score + (dc + dh) * dt;
                    const auto type_next = "wait";

                    history_.append_history(score_next, time_next, index_vertex, curr.index_history, type_next);
                    queue_.append_queue(score_next, time_next, index_vertex, index_history, n_visits_next);

                    index_history += 1;
                    map_[index_edge] += 1;

                }  else if (!is_constrained && !is_visited) { 
                    const auto n_visits_next = curr.n_visits;

                    const auto dt = actor.lookup_time(index_edge);
                    const auto time_next = curr.time + dt;

                    const auto dc = actor.lookup_cost(index_edge);
                    const auto dh = actor.lookup_heuristic(index_edge);
                    const auto score_next = curr.score + (dc + dh) * dt;
                    const auto type_next = edge.type;

                    history_.append_history(score_next, time_next, edge.index_child, curr.index_history, type_next);
                    queue_.append_queue(score_next, time_next, edge.index_child, index_history, n_visits_next);

                    index_history += 1;
                    map_[index_edge] += 1;

                } else if (curr.index_parent == actor.index_target_ && curr.time < t_expand && edge.type == "wait") {
                    const auto n_visits_next = curr.n_visits + 1;

                    const auto dt = actor.lookup_time(index_edge);
                    const auto time_next = curr.time + dt;

                    const auto dc = actor.lookup_cost(index_edge);
                    const auto dh = actor.lookup_heuristic(index_edge);
                    const auto score_next = curr.score + (dc + dh) * dt;
                    const auto type_next = edge.type;

                    history_.append_history(score_next, time_next, edge.index_child, curr.index_history, type_next);
                    queue_.append_queue(score_next, time_next, edge.index_child, index_history, n_visits_next);

                    index_history += 1;
                    map_[index_edge] += 1;

                    if (is_other_at_goal) {
                        std::cout << curr.index_parent << ", " << actor.index_target_ << std::endl;
                    }

                } else if (is_other_at_goal) {
                    continue;
                }
            }

            // if (queue_.size_queue() < 2) {
            //     index_history = history_.size_history() - 1;
            // }
        }

        low_level_search_output output;
        output.score = history_.unpack_score(index_complete);
        output.path = history_.unpack_path(index_complete);

        auto t_term = std::chrono::steady_clock::now();

        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t_term - t_init).count() << std::endl;

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

    void save_low_level_search_output(const std::string& path_goal, const low_level_search_output& path, const Graph& graph, const bool& use_simplify)
    {
        const int64_t& n_paths = static_cast<int64_t>(path.path.size());
        std::vector<graph_vertex> vertices;
        
        auto vertex_last = graph.lookup_vertex(path.path[0].second);
        vertices.push_back(vertex_last);

        auto vertex_curr = graph.lookup_vertex(path.path[1].second);
        for (int64_t iter = 1; iter < n_paths - 1; ++iter) {
            if (!use_simplify) {
                auto vertex_curr = graph.lookup_vertex(path.path[iter].second);
                vertices.push_back(vertex_curr);
            } else {

                const auto index_next = path.path[iter + 1].second;
                const auto vertex_next = graph.lookup_vertex(index_next);
                
                const bool is_same_x = (vertex_last.x == vertex_curr.x);
                const bool is_same_y = (vertex_last.y == vertex_curr.y);
                const bool is_same_theta = (vertex_last.theta == vertex_curr.theta);

                const bool will_be_same_x = (vertex_curr.x == vertex_next.x);
                const bool will_be_same_y = (vertex_curr.y == vertex_next.y);
                const bool will_be_same_theta = (vertex_curr.theta == vertex_next.theta);

                const bool is_wait = (is_same_x && is_same_y && is_same_theta);
                const bool is_rotate = (is_same_x && is_same_y && !is_same_theta);
                const bool is_translate = (is_same_theta && !is_wait && !is_rotate);

                const bool will_be_wait = (will_be_same_x && will_be_same_y && will_be_same_theta);
                const bool will_be_rotate = (will_be_same_x && will_be_same_y && !will_be_same_theta);
                const bool will_be_translate = (will_be_same_theta && !will_be_wait && !will_be_rotate);

                if (is_wait || is_rotate) {
                    vertices.push_back(vertex_curr);
                }

                if (is_translate && !will_be_translate) {
                    vertices.push_back(vertex_curr);
                }

                vertex_last = vertex_curr;
                vertex_curr = vertex_next;
            }
        }
        vertices.push_back(vertex_curr);

        std::string path_new;
        if (!path_goal.empty() && path_goal[0] == '~') {
            const char* home = getenv("HOME");
            if (home) {
                path_new = std::string(home) + path_goal.substr(1);
            } else {
                throw std::runtime_error("could not determine the home directory");
            }
        }

        std::ofstream file(path_new);
        if (!file.is_open()) {
            throw std::runtime_error("could not open file for writing");
        }

        file << "x" << ", " << "y" << ", " << "theta" << "\n";
        const int64_t& n_vertices = static_cast<int64_t>(vertices.size());
        for (int64_t iter = 0; iter < n_vertices; ++iter) {
            const auto vertex = vertices[iter];
            file << vertex.x << ", " << vertex.y << ", " << vertex.theta * (M_PI / 180) << "\n";
        }

        file.close();
    }

} // namespace mess2_algorithms
