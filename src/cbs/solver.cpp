
#include "mess2_algorithm_plugins/cbs/solver.hpp"

namespace mess2_algorithms
{
    CBS::CBS(std::shared_ptr<Instance> &_instance, bool _sipp, int _screen) : screen(_screen), focal_w(1.0), instance(_instance)
    // mdd_helper(initial_constraints, search_engines),
    // rectangle_helper(instance),
    // mutex_helper(instance, initial_constraints),
    // corridor_helper(search_engines, initial_constraints),
    // heuristic_helper(instance.getDefaultNumberOfAgents(), paths, search_engines, initial_constraints, mdd_helper)
    {
        clock_t t = std::clock();
        initial_constraints.resize(instance->n_actors, ConstraintTable());
        search_engines.resize(instance->n_actors);
        for (int iter = 0; iter < instance->n_actors; ++iter) {
            if (_sipp) {
                // search_engines[iter] == std::make_shared<SIPP>();
                throw std::runtime_error("sipp not implemented yet");
            } else {
                search_engines[iter] = std::make_shared<SpaceTimeAStar>(instance, iter);
            }
            runtime_preprocessing = (double) (std::clock() - t) / CLOCKS_PER_SEC;

            // mutex_helper.search_engines = search_engines;

            // screen for debugging omitted
        }
    };


    bool CBS::generate_root()
    {
        dummy_start = std::make_shared<CBSNode>();
        dummy_start->g_cummulative = 0.0;
        paths.resize(instance->n_actors, nullptr);

        // mdd_helper.init(num_of_agents);
    	// heuristic_helper.init();

        if (paths_found_initially.empty())
        {
            paths_found_initially.resize(instance->n_actors);

            // generate a random permutation of actor instances
            std::vector<int> actors(instance->n_actors);
            for (int iter = 0; iter < instance->n_actors; ++iter) {
                actors[iter] = iter;
            }

            if (random_root) {
                std::random_device rd;
                std::mt19937 g(rd());
                std::shuffle(std::begin(actors), std::end(actors), g);
            }

            for (auto i : actors) {
                paths_found_initially[i] = search_engines[i]->find_path(dummy_start, initial_constraints[i], paths, 0.0);

                if (paths_found_initially[i].empty()) {
                    std::cout << "CBS::generate_root : no path exists for actor " << i << std::endl;
                    return false;
                }
                paths[i] = std::make_shared<Path>(paths_found_initially[i]);
                dummy_start->makespan = std::max(dummy_start->makespan, paths_found_initially[i].size());
                dummy_start->g_cummulative += (double) paths_found_initially[i][paths_found_initially[i].size() - 1].score;
                n_ll_expanded += search_engines[i]->n_expanded;
                n_ll_generated += search_engines[i]->n_generated;
            }
        } else {
            for (auto i = 0; i < instance->n_actors; ++i) {
                paths[i] = std::make_shared<Path>(paths_found_initially[i]);
                dummy_start->makespan = std::max(dummy_start->makespan, paths_found_initially[i].size());
                dummy_start->g_cummulative += (double) paths_found_initially[i][paths_found_initially[i].size() - 1].score;
            }
        }

        dummy_start->h_cummulative = 0.0;
        dummy_start->handle_open = list_open.push(dummy_start);
        dummy_start->handle_focal = list_focal.push(dummy_start);

        n_hl_generated += 1;
        dummy_start->time_generated = n_hl_generated;
        table_of_all_nodes.push_back(dummy_start);
        find_conflicts(dummy_start);
        score_min = std::max(score_min, (double) dummy_start->g_cummulative);
        threshold_list_focal = score_min * focal_w;

        // remove logic about printing for debugging

        return true;
    }


    void CBS::copy_conflicts(const std::list<std::shared_ptr<Conflict>> &_conflicts, std::list<std::shared_ptr<Conflict>> &_copy, const std::list<int> &_actors_excluded)
    {
        for (const auto &conflict : _conflicts) {
            bool found = false;
            for (auto a : _actors_excluded) {
                if (conflict->index_actor1 == a || conflict->index_actor2 == a) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                assert(!conflict->constraint1.empty());
                assert(!conflict->constraint2.empty());
                _copy.push_back(conflict);
            }
        }
    }


    bool CBS::find_collisions(const std::shared_ptr<Key3D> &_key1, const std::shared_ptr<Key3D> &_key2, const double &_radius)
    {
        auto p1 = instance->graph->lookup_point(_key1->index_key);
        auto p2 = instance->graph->lookup_point(_key2->index_key);
        auto dx = std::abs(*(p1->x) - *(p2->x));
        auto dy = std::abs(*(p1->y) - *(p2->y));
        auto dz = std::abs(*(p1->z) - *(p2->z));
        auto distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance <= _radius) {
            std::cout << "GOTCHA" << std::endl;
            return true;
        }
        return false;
    }


    void CBS::find_conflicts(std::shared_ptr<CBSNode>& _node, int &_index_actor1, int &_index_actor2)
    {
        auto actor1 = instance->actors[_index_actor1];
        auto actor2 = instance->actors[_index_actor2];
        auto path1 = *paths[_index_actor1];
        auto path2 = *paths[_index_actor2];
        auto counter = std::pair<int, int>(0, 0);
        auto sizes = std::pair<int, int>(static_cast<int>(path1.size()) - 1, static_cast<int>(path2.size()) - 1);

        std::pair<std::list<std::shared_ptr<Key3D>>, std::list<std::shared_ptr<Key3D>>> occupancies;

        auto key1 = instance->graph->lookup_key_by_index_vertex(path1[counter.first].index_vertex);
        occupancies.first = actor1->lookup_occupancies_symbolically(instance->graph, key1->i, key1->j, key1->k, actor1->occupancies_symbolic);
        counter.first += 1;

        auto key2 = instance->graph->lookup_key_by_index_vertex(path2[counter.second].index_vertex);
        occupancies.second = actor2->lookup_occupancies_symbolically(instance->graph, key2->i, key2->j, key2->k, actor2->occupancies_symbolic);
        counter.second += 1;
        
        double t1 = 0.0;
        double t2 = 0.0;
        std::shared_ptr<Vertex> vertex1;
        std::shared_ptr<Vertex> vertex2;

        while (counter != sizes) {
            // select next lowest time instance
            t1 = path1[counter.first].time;
            t2 = path2[counter.second].time;
            
            // update second actor location and conflictsz
            if (t2 < t1 && counter.second != sizes.second) {
                vertex2 = instance->graph->lookup_vertex(path2[counter.second].index_vertex);
                key2 = vertex2->point->key;
                occupancies.second = actor2->lookup_occupancies_symbolically(instance->graph, key2->i, key2->j, key2->k, actor2->occupancies_symbolic);
                if (counter.second < sizes.second) {
                    counter.second += 1;
                }

            // update first actor
            } else {
                vertex1 = instance->graph->lookup_vertex(path1[counter.first].index_vertex);
                key1 = vertex1->point->key;
                occupancies.first = actor1->lookup_occupancies_symbolically(instance->graph, key1->i, key1->j, key1->k, actor1->occupancies_symbolic);
                if (counter.first < sizes.first) {
                    counter.first += 1;
                }
            }

            // determine if any collisions occur
            bool is_conflicting = find_collisions(key1, key2, actor1->radius + actor2->radius);

            // if collisions occur ...
            if (is_conflicting) {
                auto conflict = std::make_shared<Conflict>();
                // first actor reached target
                if (target_reasoning && counter.first == sizes.first) {
                    conflict->define_as_target(_index_actor1, _index_actor2, vertex1->index_vertex, path1[counter.first - 1].time, path1[counter.first].time, path1[counter.first].time);

                // second actor reached target 
                } else if (target_reasoning && counter.second == sizes.second) {
                    conflict->define_as_target(_index_actor2, _index_actor1, vertex2->index_vertex, path2[counter.second - 1].time, path2[counter.second].time, path2[counter.second].time);

                // neither actor at target, define as point (multi point additions in ct build so only provide single index here (key indices = point indices given graph generation nature))
                } else {
                    auto i11 = std::max(int(0), counter.first - 2);
                    auto i12 = std::min(counter.first, sizes.first);
                    auto i21 = std::max(int(0), counter.second - 2);
                    auto i22 = std::min(counter.second, sizes.second);

                    auto t11 = path1[i11].time;
                    auto t12 = path1[i12].time;
                    auto t21 = path2[i21].time;
                    auto t22 = path2[i22].time;
                    conflict->define_as_point(_index_actor1, _index_actor2, key1->index_key, key2->index_key, t11, t12, t21, t22, actor1->radius + actor2->radius);
                }
                assert(!conflict->constraint1.empty());
                assert(!conflict->constraint2.empty());
                _node->conflicts_unknown.push_back(conflict);

            // edge conflict (likely won't occur in high density graph, but in graph where actor collisions can transition along the same edge without occupying conflicting points, edge conflicts may exist)
            } else if (counter.first - 1 < sizes.first - 1 && counter.second - 1 < sizes.second && key1->index_key == path2[counter.second - 1].index_vertex && key2->index_key == path1[counter.first - 1].index_vertex) {
                auto conflict = std::make_shared<Conflict>();
                auto edge1 = instance->graph->find_edge_by_vertex_indices(key1->index_key, key2->index_key); // edge actor 1 takes
                auto edge2 = instance->graph->find_edge_by_vertex_indices(key2->index_key, key1->index_key); // edge actor 2 takes

                auto i11 = std::max(int(0), counter.first - 2);
                auto i12 = std::min(counter.first, sizes.first);
                auto i21 = std::max(int(0), counter.second - 2);
                auto i22 = std::min(counter.second, sizes.second);

                auto t11 = path1[i11].time;
                auto t12 = path1[i12].time;
                auto t21 = path2[i21].time;
                auto t22 = path2[i22].time;

                t1 = 0.0;
                t2 = 0.0;
                auto edges11 = instance->graph->lookup_edges(edge1->vertex_parent->index_vertex);
                auto edges12 = instance->graph->lookup_edges(edge1->vertex_child->index_vertex);
                auto edges21 = instance->graph->lookup_edges(edge2->vertex_parent->index_vertex);
                auto edges22 = instance->graph->lookup_edges(edge2->vertex_child->index_vertex);
                for (const auto &edge11 : edges11) {
                    if (actor1->lookup_t(edge11->index_edge) > t1) {
                        t1 = actor1->lookup_t(edge11->index_edge);
                    }
                }
                for (const auto &edge12 : edges12) {
                    if (actor1->lookup_t(edge12->index_edge) > t1) {
                        t1 = actor1->lookup_t(edge12->index_edge);
                    }
                }
                for (const auto &edge21 : edges21) {
                    if (actor2->lookup_t(edge21->index_edge) > t1) {
                        t2 = actor2->lookup_t(edge21->index_edge);
                    }
                }
                for (const auto &edge22 : edges22) {
                    if (actor2->lookup_t(edge22->index_edge) > t1) {
                        t2 = actor2->lookup_t(edge22->index_edge);
                    }
                }
                
                conflict->define_as_edge(_index_actor1, _index_actor2, edge1->index_edge, t11, t12, t21, t22, t1, t2);
                assert(!conflict->constraint1.empty());
                assert(!conflict->constraint2.empty());
                _node->conflicts_unknown.push_back(conflict);
            }
        } 
    }


    void CBS::find_conflicts(std::shared_ptr<CBSNode>& _node)
    {
        clock_t t = std::clock();
        if (_node->parent != nullptr) {
            std::list<int> actors_new;
            for (const auto &p : _node->paths) {
                actors_new.push_back(p.first);
            }
            (void) copy_conflicts(_node->parent->conflicts_known, _node->conflicts_known, actors_new);
            (void) copy_conflicts(_node->parent->conflicts_unknown, _node->conflicts_unknown, actors_new);

            for (auto i = actors_new.begin(); i != actors_new.end(); ++i) {
                int index_actor1 = *i;
                for (auto index_actor2 = 0; index_actor2 < instance->n_actors; ++index_actor2) {
                    if (index_actor1 == index_actor2) {
                        continue;
                    }
                    bool skip = false;
                    for (auto j = actors_new.begin(); j != i; ++j) {
                        if (*j == index_actor2) {
                            skip = true;
                            break;
                        }
                    }
                    if (!skip) {
                        find_conflicts(_node, index_actor1, index_actor2);
                    }
                }
            }
        } else {
            for (auto index_actor1 = 0; index_actor1 < instance->n_actors; ++index_actor1) {
                for (auto index_actor2 = index_actor1 + 1; index_actor2 < instance->n_actors; ++index_actor2) {
                    find_conflicts(_node, index_actor1, index_actor2);
                }
            }
        }
    }


    bool CBS::solve(double _time_limit, double _cost_lowerbound, double _cost_upperbound)
    {
        this->score_min = _cost_lowerbound;
        this->cost_upperbound = _cost_upperbound;
        this->time_limit = _time_limit;

        // screen for debugging omitted

        time_start = std::clock();

        (void) generate_root();

        for (const auto &path : paths) {
            std::cout << "g : " << path->back().cost << ", h : " << path->back().heuristic << std::endl;
        }





        return false;
    }


    void CBS::save_paths(const std::string &_path_goals, bool simplify)
    {
        for (auto i = 0; i < instance->n_actors; ++i) {
            auto path = paths[i];
            std::stringstream ss;
            ss << i + 1;
            std::string index_actor = ss.str();

            std::vector<std::shared_ptr<Vertex>> vertices;
            std::vector<std::tuple<double, double, double, double>> data;

            auto n_path = static_cast<int>(path->size());
            for (auto j = 0; j < n_path; ++j) {
                vertices.push_back(instance->graph->lookup_vertex((*path)[j].index_vertex));
            }

            if (!simplify) {
                std::shared_ptr<Vertex> curr;
                for (auto j = 0; j < n_path; ++j) {
                    curr = vertices[j];
                    data.emplace_back(*(curr->point->x), *(curr->point->y), *(curr->point->z), curr->heading);
                }
            } else {
                std::shared_ptr<Vertex> prev;
                std::shared_ptr<Vertex> curr;
                std::shared_ptr<Vertex> next;
                bool is_same_x, is_same_y, is_same_z, will_be_same_x, will_be_same_y, will_be_same_z, is_wait, is_rotate, is_translate_in_plane, is_translate_out_of_plane, will_be_wait, will_be_rotate, will_be_translate_in_plane, will_be_translate_out_of_plane, is_same_theta, will_be_same_theta;
                
                data.emplace_back(*(vertices[0]->point->x), *(vertices[0]->point->y), *(vertices[0]->point->z), vertices[0]->heading);
                for (auto j = 1; j < n_path - 1; ++j) {
                    prev = vertices[j - 1];
                    curr = vertices[j];
                    next = vertices[j + 1];

                    is_same_x = (prev->point->x == curr->point->x);
                    is_same_y = (prev->point->y == curr->point->y);
                    is_same_z = (prev->point->z == curr->point->z);
                    is_same_theta = (prev->heading == curr->heading);

                    will_be_same_x = (next->point->x == curr->point->x);
                    will_be_same_y = (next->point->y == curr->point->y);
                    will_be_same_z = (next->point->z == curr->point->z);
                    will_be_same_theta = (next->heading == curr->heading);

                    is_wait = (is_same_x && is_same_y && is_same_theta);
                    is_rotate = (is_same_x && is_same_y && !is_same_theta);
                    is_translate_in_plane = (is_same_theta && !is_wait && !is_rotate && is_same_z);
                    is_translate_out_of_plane = (is_same_theta && !is_wait && !is_rotate && is_same_x && is_same_y);

                    will_be_wait = (will_be_same_x && will_be_same_y && will_be_same_theta);
                    will_be_rotate = (will_be_same_x && will_be_same_y && !will_be_same_theta);
                    will_be_translate_in_plane = (will_be_same_theta && !will_be_wait && !will_be_rotate && will_be_same_z);
                    will_be_translate_out_of_plane = (will_be_same_theta && !will_be_wait && !will_be_rotate && will_be_same_x && will_be_same_y);

                    if (is_wait || is_rotate) {
                        data.emplace_back(*(curr->point->x), *(curr->point->y), *(curr->point->z), curr->heading);
                    } 
                    
                    if (is_translate_in_plane && !will_be_translate_in_plane) {
                        data.emplace_back(*(curr->point->x), *(curr->point->y), *(curr->point->z), curr->heading);
                    }

                    if (is_translate_out_of_plane && !will_be_translate_out_of_plane) {
                        data.emplace_back(*(curr->point->x), *(curr->point->y), *(curr->point->z), curr->heading);
                    }
                }
                data.emplace_back(*(vertices.back()->point->x), *(vertices.back()->point->y), *(vertices.back()->point->z), vertices.back()->heading);
            }

            auto path_goal = _path_goals + "actor" + index_actor + ".csv";
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

            file << "x" << ", " << "y" << ", " << "z" << ", " << "theta" << "\n";
            double x, y, z, theta;
            for (auto line : data) {
                x = std::get<0>(line);
                y = std::get<1>(line);
                z = std::get<2>(line);
                theta = std::get<3>(line);
                file << x << ", " << y << ", " << z << ", " << theta * (M_PI / 180) << "\n";
            }

            file.close();
        }
    }

} // namespace mess2_algorithms
