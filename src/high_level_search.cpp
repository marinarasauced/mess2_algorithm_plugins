
#include "mess2_algorithm_plugins/high_level_search.hpp"

namespace mess2_algorithms
{
    HighLevelSearch::HighLevelSearch() {};

    high_level_search_output HighLevelSearch::execute_high_level_search(const Graph& graph, std::vector<Actor>& actors, const double& timeout)
    {
        const int64_t n_actors = static_cast<int64_t>(actors.size());
        std::vector<ConstraintsVertices> constraints;
        constraints.reserve(n_actors);
        for (int64_t iter = 0; iter < n_actors; ++iter) {
            constraints.emplace_back(ConstraintsVertices(graph.n_vertices));
        }

        (void) queue_.clear_queue();
        (void) queue_.append_queue(0.0, constraints);

        const auto t_init = std::chrono::steady_clock::now();
        while (queue_.size_queue() > 0)
        // for (int64_t kter = 0; kter < 2; ++kter)
        {
            bool is_timeout = (std::chrono::duration<double>(std::chrono::steady_clock::now() - t_init).count() > timeout);
            if (is_timeout) {
                throw std::runtime_error("high level search timed out");
                break;
            }

            double scores = 0.0;
            std::vector<std::vector<std::pair<double, int64_t>>> paths;
            paths.reserve(n_actors);

            const auto curr = queue_.lookup_queue();

            for (int64_t iter = 0; iter < n_actors; ++iter) {
                low_level_search_output output_low = search_.execute_low_level_search(graph, actors[iter], curr.constraints[iter]);

                scores += output_low.score;
                paths.push_back(output_low.path);
                (void) info_low_level_search_output(output_low, graph, actors[iter]);
            }

            bool is_conflicted = execute_conflict_search(scores, paths, graph, actors, constraints);
            if (!is_conflicted) { break; }
        }

        high_level_search_output output_high;
        return output_high;
    }

    bool HighLevelSearch::execute_conflict_search(const double& score, const std::vector<std::vector<std::pair<double, int64_t>>>& paths, const Graph& graph, std::vector<Actor>& actors, std::vector<ConstraintsVertices>& constraints)
    {
        const int64_t n_paths = static_cast<int64_t>(paths.size());
        std::vector<int64_t> indices_max(n_paths);
        std::vector<std::pair<double, double>> bounds(n_paths);
        for (int64_t iter = 0; iter < n_paths; ++iter) {
            std::pair<double, double> bound;
            const auto path = paths[iter];
            const int64_t n_path = static_cast<int64_t>(path.size());
            indices_max[iter] = n_path;
            bound.first = path[0].first;
            bound.second = path[n_path - 1].first;
            bounds[iter] = bound;
        }

        std::vector<std::vector<int64_t>> occupancies(n_paths);
        for (int64_t iter = 0; iter < n_paths; ++iter) {
            const auto vertex = graph.lookup_vertex(paths[iter][0].second);
            occupancies[iter] = actors[iter].lookup_occupancies(vertex.x, vertex.y);
        }

        std::vector<int64_t> indices_curr(n_paths, 1);
        while (indices_curr != indices_max) {
            double time_next = std::numeric_limits<double>::max();
            int64_t index_actor = -1;
            for (int64_t iter = 0; iter < n_paths; ++iter) {
                const auto index_curr = indices_curr[iter];
                if (index_curr >= indices_max[iter]) {
                    continue;
                }

                const auto time_curr = paths[iter][index_curr].first;
                if (time_curr < time_next) {
                    time_next = time_curr;
                    index_actor = iter;
                }
            }

            const auto vertex = graph.lookup_vertex(paths[index_actor][indices_curr[index_actor]].second);
            // std::cout << vertex.x << ", " << vertex.y << std::endl;
            occupancies[index_actor] = actors[index_actor].lookup_occupancies(vertex.x, vertex.y);
            arma::vec counter = arma::zeros(graph.n_vertices);

            bool is_conflicted = false;
            for (int64_t iter = 0; iter < n_paths; ++iter) {
                // std::cout << iter << ", " << occupancies[iter].size() << " : ";
                for (const auto& index : occupancies[iter]) {
                    // std::cout << index << ", ";
                    counter(index) += 1;
                    if (counter(index) > 1) {
                        auto index_min = std::max(int64_t(0), indices_curr[iter] - 1);
                        auto index_max = std::min(indices_curr[iter] + 1, indices_max[iter]);

                        auto time_min = paths[iter][index_min].first;
                        auto time_max = paths[iter][index_max].first;
                        // std::cout << paths[iter][indices_curr[iter]].first << " adding " << time_min << " to " << time_max << std::endl;
                        if (index_max == indices_max[iter]) {
                            time_max = std::numeric_limits<double>::max();
                        }

                        constraints[iter].constrain_vertex(index, time_min, time_max);
                        std::cout << "conflict detected, adding constraint at " << index << " from " << time_min << " to " << time_max << std::endl;
                        is_conflicted = true;
                    }
                }
                // std::cout << std::endl;
            }

            if (is_conflicted) {
                (void) execute_conflict_resolution(score, constraints);
                return true;
            }
            indices_curr[index_actor] += 1;
        }
        return false;
    }

    void HighLevelSearch::execute_conflict_resolution(const double& score, const std::vector<ConstraintsVertices>& constraints)
    {
        queue_.append_queue(score, constraints); 
    }

    void HighLevelSearch::info()
    {
        throw std::runtime_error("high level search info not implemented");
    }

} // namespace mess2_algorithms
