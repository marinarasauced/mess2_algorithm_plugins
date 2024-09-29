
#include "mess2_algorithm_plugins/high_level_search.hpp"

namespace mess2_algorithms
{
    HighLevelSearch::HighLevelSearch() {};

    high_level_search_output HighLevelSearch::execute_high_level_search(const Graph& graph, std::vector<Actor>& actors, const int64_t& n_iters)
    {
        const int64_t n_actors = static_cast<int64_t>(actors.size());
        std::vector<ConstraintsVertices> constraints;
        constraints.reserve(n_actors);
        for (int64_t iter = 0; iter < n_actors; ++iter) {
            constraints.emplace_back(ConstraintsVertices(graph.n_vertices));
        }

        (void) queue_.clear_queue();
        (void) queue_.append_queue(0, 0.0, constraints);

        double scores;
        high_level_search_output paths;
        int64_t n_iter = 0;
        // while (queue_.size_queue() > 0)
        for (int64_t kter = 0; kter < 9900; ++kter)
        {
            bool is_timedout = (n_iter > n_iters);
            if (is_timedout) {
                // throw std::runtime_error("high level search timed out by exceeding max number of iterations");
                // break;
            }
            n_iter += 1;
            std::cout << "iter # " << n_iter << std::endl;

            scores = 0.0;
            paths.clear();

            const auto curr = queue_.lookup_queue();
            constraints = curr.constraints;

            for (int64_t iter = 0; iter < n_actors; ++iter) {
                low_level_search_output output_low = search_.execute_low_level_search(graph, actors[iter], curr.constraints[iter]);

                scores += output_low.score;
                paths.push_back({output_low.score, output_low.path});
                // (void) info_low_level_search_output(output_low, graph, actors[iter]);
            }

            bool is_conflicted = execute_conflict_search(scores, paths, graph, actors, constraints, curr.n_constraints);
            if (!is_conflicted) { break; }
        }

        high_level_search_output output_high;
        output_high = paths;
        return output_high;
    }

    bool HighLevelSearch::execute_conflict_search(const double& score, const high_level_search_output& paths, const Graph& graph, std::vector<Actor>& actors, std::vector<ConstraintsVertices>& cts, const int64_t& n_constraints)
    {
        const auto n_actors = static_cast<int64_t>(paths.size());

        std::vector<int64_t> indices_curr(n_actors, 0);
        std::vector<int64_t> indices_max(n_actors, 0);
        std::vector<std::vector<int64_t>> occupancies(n_actors);

        for (int64_t iter = 0; iter < n_actors; ++iter) {
            const auto index_curr = indices_curr[iter];
            const auto path = paths[iter].path;
            const auto vertex_curr = graph.lookup_vertex(path[index_curr].second);
            
            indices_curr[iter] += 1;
            indices_max[iter] = static_cast<int64_t>(path.size());
            occupancies[iter] = actors[iter].lookup_occupancies(vertex_curr.x, vertex_curr.y);
        }

        while (indices_curr != indices_max) {

            // determine index of next actor using smallest time step or first come first serve if tie
            int64_t index_actor = -1;
            double time_next = std::numeric_limits<double>::max();
            for (int64_t iter = 0; iter < n_actors; ++iter) {
                const auto index_curr = indices_curr[iter];
                if (index_curr >= indices_max[iter]) {
                    continue;
                }
                const auto path_curr = paths[iter].path[index_curr];
                const auto time_curr = path_curr.first;
                if (time_curr < time_next) {
                    index_actor = iter;
                    time_next = time_curr;
                }
            }

            // update occupancies of current actor
            const auto index_curr = indices_curr[index_actor];
            const auto path_curr = paths[index_actor].path[index_curr];
            const auto vertex_curr = graph.lookup_vertex(path_curr.second);
            occupancies[index_actor] = actors[index_actor].lookup_occupancies(vertex_curr.x, vertex_curr.y);

            // compare current actor's occupanices to other actors' occupancies
            const std::unordered_set<int64_t> keys(occupancies[index_actor].begin(), occupancies[index_actor].end());
            int64_t index_other = -1;
            for (int64_t iter = 0; iter < n_actors; ++iter) {
                if (iter == index_actor) {
                    continue;
                }

                for (const auto& occupancy : occupancies[iter]) {
                    if (keys.find(occupancy) != keys.end()) {
                        index_other = iter;
                        break;
                    }
                }

                if (index_other != -1) {
                    break;
                }
            }

            if (index_other != -1) {
                
                // split constraint tree entry for new branches
                auto cts_actor = cts;
                auto cts_other = cts;

                // retrieve vertices of conflicting actors poses
                auto vertex_actor = graph.lookup_vertex(path_curr.second);
                auto vertex_other = graph.lookup_vertex(paths[index_other].path[indices_curr[index_other]].second);

                // retrieve indices vertices of (x, y) coordinate associated with above vertices
                auto indices_actor = graph.lookup_map(vertex_actor.x, vertex_actor.y);
                auto indices_other = graph.lookup_map(vertex_other.x, vertex_other.y);

                // find time constraints
                int64_t index_t_min = std::max(int64_t(0), indices_curr[index_other] - 1); // other always gets to conflict before actor
                double t_min = paths[index_other].path[index_t_min].first;

                int64_t index_t_max_other = std::min(int64_t(paths[index_other].path.size()), indices_curr[index_actor] + 1);
                int64_t index_t_max_actor = std::min(int64_t(paths[index_other].path.size()), indices_curr[index_other] + 1);
                double t_max = std::max(paths[index_other].path[index_t_max_other].first, paths[index_actor].path[index_t_max_actor].first);

                std::cout << "conflict detected between actors " << index_actor << " & " << index_other << " at (" << vertex_curr.x << ", " << vertex_curr.y << ") from " << t_min << " to " << t_max << std::endl;

                // populate new constraints for actor and other
                for (const auto& index : indices_actor) {
                    cts_other[index_actor].constrain_vertex(index, t_min, t_max);
                }
                for (const auto& index : indices_other) {
                    cts_actor[index_other].constrain_vertex(index, t_min, t_max);
                }

                // append queue with new branches
                // (void) execute_conflict_resolution(n_constraints + 1, score, cts_actor);
                (void) execute_conflict_resolution(n_constraints + 1, score, cts_other);

                return true;
            }
            
            // const auto path = paths[iter].path;
            // const auto vertex_curr = graph.lookup_vertex(path[index_curr].second);
            
            // indices_curr[iter] += 1;
            // indices_max[iter] = static_cast<int64_t>(path.size());
            // occupancies[iter] = actors[iter].lookup_occupancies(vertex_curr.x, vertex_curr.y);


            indices_curr[index_actor] += 1;
        }

        // // std::vector<int64_t> indices_curr(n_paths, 1);
        // // while (indices_curr != indices_max) {
        // //     double time_next = std::numeric_limits<double>::max();
        // //     int64_t index_actor = -1;
        // //     for (int64_t iter = 0; iter < n_paths; ++iter) {
        // //         const auto index_curr = indices_curr[iter];
        // //         if (index_curr >= indices_max[iter]) {
        // //             continue;
        // //         }

        // //         const auto time_curr = paths[iter].path[index_curr].first;
        // //         if (time_curr < time_next) {
        // //             time_next = time_curr;
        // //             index_actor = iter;
        // //         }
        // //     }

        // //     const auto vertex = graph.lookup_vertex(paths[index_actor].path[indices_curr[index_actor]].second);
        // //     occupancies[index_actor] = actors[index_actor].lookup_occupancies(vertex.x, vertex.y);
        // //     arma::vec counter = arma::zeros(graph.n_vertices);

        // //     bool is_conflicted = false;
        // //     int64_t index_actor_conflicted = -1;
        // //     auto cts_actor = cts;
        // //     auto cts2_actor_conflicted = cts;

        // //     int64_t index_t_min;
        // //     int64_t index_t_max;

        // //     double t_min_constraint;
        // //     double t_max_constraint;

        // //     for (int64_t iter = 0; iter < n_paths; ++iter) {
        // //         for (const auto& index : occupancies[iter]) {
        // //             counter(index) += 1;
        // //             if (counter(index) > 1) {
        // //                 index_actor_conflicted = iter;
        // //                 is_conflicted = true;
        // //                 cts_actor[index_actor].constrain_vertex(index, t_min_constraint, t_max_constraint);
        // //                 cts2_actor_conflicted[index_actor_conflicted].constrain_vertex(index, t_min_constraint, t_max_constraint);
        // //                 n_constraint += 1;
        // //             }
        // //         }
        // //         if (is_conflicted) {
        // //             break;
        // //         }
        // //     }
        //     // std::cout << index_actor << " & " << index_actor_conflicted << std::endl;
        //     // // for (const auto& index : occupancies[index_actor]) {
        //     // //     if (counter[index] > 1) {
        //     // //         is_conflicted = true;
        //     // //     }
        //     // // }

        //     // bool added_constraints = false;
        //     // auto cts1 = cts;
        //     // auto cts2 = cts;
        //     // if (!is_conflicted) {

        //     // } else if (is_conflicted) {
        //     //     for (int64_t iter = 0; iter < n_paths; ++iter) {
        //     //         if (iter != index_actor) {
        //     //             std::cout << "actor : " << iter << std::endl;
        //     //         //     for (const auto& index : occupancies[iter]) {
        //     //         //         if (counter[index] > 1) {
        //     //         //             std::cout << index_actor << " & " << iter << " constrained at (" << graph.lookup_vertex(index).x << ", " << graph.lookup_vertex(index).y << ", " << graph.lookup_vertex(index).theta << ")" << std::endl;
        //     //         //             added_constraints = true;
        //     //         //         }
        //     //         //         // if (std::find(indices_constraints.begin(), indices_constraints.end(), index) != indices_constraints.end()) {
        //     //         //         //     std::cout << index_actor << " & " << iter << " constrained at (" << graph.lookup_vertex(index).x << ", " << graph.lookup_vertex(index).y << ", " << graph.lookup_vertex(index).theta << ")" << std::endl;
        //     //         //         //     added_constraints = true;
        //     //         //         // }
        //     //         //     }
        //     //         // }

        //     //         // if (added_constraints) {
        //     //         //     break;
        //     //         // }
        //     //         }
        //     //     }
        //     // }

        //     if (is_conflicted) {
        //         break;
        //     }

        //     // if (is_conflicted) {
        //     //     (void) execute_conflict_resolution(n_constraints_, score, constraints);
        //     //     return true;
        //     // }
        //     indices_curr[index_actor] += 1;
        // }
        return false;
    }

    void HighLevelSearch::execute_conflict_resolution(const int64_t& n_constraints, const double& score, const std::vector<ConstraintsVertices>& constraints)
    {
        queue_.append_queue(n_constraints, score, constraints); 
    }

    void HighLevelSearch::info()
    {
        throw std::runtime_error("high level search info not implemented");
    }

    void save_high_level_search_output(const std::string& path_goals, const high_level_search_output& paths, const Graph& graph, const bool& use_simplify)
    {
        const int64_t& n_paths = static_cast<int64_t>(paths.size());
        for (int64_t iter = 0; iter < n_paths; ++iter) {
            const low_level_search_output path = paths[iter];
            std::stringstream ss;
            ss << iter + 1;
            std::string index_actor = ss.str();
            (void) save_low_level_search_output(path_goals + "actor" + index_actor + ".csv", path, graph, use_simplify);
        }
    }

} // namespace mess2_algorithms
