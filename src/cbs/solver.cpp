
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
        initial_constraints.resize(instance->n_actors, ConstraintTable(instance));
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
        paths.resize(instance->n_actors);

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

            for (auto iter = 0; iter < instance->n_actors; ++iter) {
                auto i = actors[iter]; // index of the current actor

                if (use_cat) {
                    for (auto jter = iter + 1; jter < instance->n_actors; ++jter) {
                        auto j = actors[jter]; // index of an actor subsequent to the current actor
                        std::cout << i << j << std::endl;

                        auto actor_temp = instance->lookup_actor(j);
                        auto vertex_source_temp = instance->graph->lookup_vertex(actor_temp->index_source);
                        auto key_temp = vertex_source_temp->point->key;

                        auto occupancies = actor_temp->lookup_occupancies_symbolically(instance->graph, key_temp->i, key_temp->j, key_temp->k, actor_temp->occupancies_symbolic);

                        auto time_min = MIN_TIMESTEP;
                        auto time_max = MAX_TIMESTEP;

                        for (const auto &occupancy : occupancies) {
                            initial_constraints[i].insert_to_ct(occupancy->index_key, time_min, time_max);
                            auto point = instance->graph->lookup_point(occupancy->index_key);
                            std::cout << "added at (" << *(point->x) << ", " << *(point->y) << ", " << *(point->z) << ")" << std::endl;
                        }
                    }
                }

                paths_found_initially[i] = search_engines[i]->find_path(dummy_start, initial_constraints[i], paths, 0.0, use_cat);

                if (paths_found_initially[i].empty()) {
                    std::cout << "CBS::generate_root : no path exists for actor " << i << std::endl;
                    return false;
                }
                paths[i] = paths_found_initially[i];
                dummy_start->makespan = std::max(dummy_start->makespan, paths_found_initially[i].size());
                dummy_start->g_cummulative += (double) paths_found_initially[i][paths_found_initially[i].size() - 1].score;
                n_ll_expanded += search_engines[i]->n_expanded;
                n_ll_generated += search_engines[i]->n_generated;
            }
        } else {
            for (auto i = 0; i < instance->n_actors; ++i) {
                paths[i] = paths_found_initially[i];
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
        (void) find_conflicts(dummy_start);
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
            // std::cout << "GOTCHA" << std::endl;
            return true;
        }
        return false;
    }


    void CBS::find_conflicts(std::shared_ptr<CBSNode>& _node, int &_index_actor1, int &_index_actor2)
    {
        auto actor1 = instance->actors[_index_actor1];
        auto actor2 = instance->actors[_index_actor2];
        auto path1 = paths[_index_actor1];
        auto path2 = paths[_index_actor2];
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
        bool is_done1 = false;
        bool is_done2 = false;

        while (counter != sizes) {
            // select next lowest time instance
            t1 = path1[counter.first].time;
            t2 = path2[counter.second].time;

            // std::cout << t1 << "(" << counter.first << "/" << sizes.first << ")" << " | " << t2 << "(" << counter.second << "/" << sizes.second << ")" << std::endl;


            if (t1 <= t2 && !is_done1) {
                vertex1 = instance->graph->lookup_vertex(path1[counter.first].index_vertex);
                key1 = vertex1->point->key;
                occupancies.first = actor1->lookup_occupancies_symbolically(instance->graph, key1->i, key1->j, key1->k, actor1->occupancies_symbolic);
                if (counter.first < sizes.first) {
                    counter.first += 1;
                }
            } else if (t2 < t1 && !is_done2) {
                vertex2 = instance->graph->lookup_vertex(path2[counter.second].index_vertex);
                key2 = vertex2->point->key;
                occupancies.second = actor2->lookup_occupancies_symbolically(instance->graph, key2->i, key2->j, key2->k, actor2->occupancies_symbolic);
                if (counter.second < sizes.second) {
                    counter.second += 1;
                }
            } else if (t1 <= t2 && is_done1 && !is_done2) {
                vertex2 = instance->graph->lookup_vertex(path2[counter.second].index_vertex);
                key2 = vertex2->point->key;
                occupancies.second = actor2->lookup_occupancies_symbolically(instance->graph, key2->i, key2->j, key2->k, actor2->occupancies_symbolic);
                if (counter.second < sizes.second) {
                    counter.second += 1;
                }
            } else if (t2 < t1 && !is_done1 && is_done2) {
                vertex1 = instance->graph->lookup_vertex(path1[counter.first].index_vertex);
                key1 = vertex1->point->key;
                occupancies.first = actor1->lookup_occupancies_symbolically(instance->graph, key1->i, key1->j, key1->k, actor1->occupancies_symbolic);
                if (counter.first < sizes.first) {
                    counter.first += 1;
                }
            } else {
                //
            }

            if (!is_done1 && counter.first == sizes.first) {
                is_done1 = true;
            }
            if (!is_done2 && counter.second == sizes.second) {
                is_done2 = true;
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


    void CBS::update_list_focal()
    {
        auto head_open = list_open.top();
        if (head_open->g_cummulative + head_open->h_cummulative > score_min)
        {
            score_min = head_open->g_cummulative + head_open->h_cummulative;
            double threshold_list_focal_new = score_min * focal_w;
            for (auto n : list_open)
            {
                if (n->g_cummulative + n->h_cummulative > threshold_list_focal && n->g_cummulative + n->h_cummulative <= threshold_list_focal_new) {
                    n->handle_focal = list_focal.push(n);
                }
                threshold_list_focal = threshold_list_focal_new;
            }
        }
    }


    bool CBS::generate_child(std::shared_ptr<CBSNode> &_node, const std::shared_ptr<CBSNode> &_parent)
    {
        clock_t t1 = std::clock();
        _node->parent = _parent;
        _node->g_cummulative = _parent->g_cummulative;
        _node->makespan = _parent->makespan;
        _node->depth = _parent->depth + 1;
        int i1, i2;
        double t1, t2, tX;
        constraint_type type;
        assert(_node->constraints.size() > 0);
        std::tie(i1, i2, t1, t2, tX, type) = _node->constraints.front();

        if (type == constraint_type::LEQLENGTH) {
            // if ((int)node->constraints.size() == 2) // generated by corridor-target conflict
            // {
            //     int a = get<0>(node->constraints.back()); // it is a G-length constraint or a range constraint on this agent
            //     int lowerbound = (int)paths[a]->size() - 1;
            //     if (!findPathForSingleAgent(node, a, lowerbound))
            //     {
            //         runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
            //         return false;
            //     }
            // }
            for (auto index_actor = 0; index_actor < instance->n_actors; ++index_actor) {
                if (i1 == index_actor) {
                    continue;
                }
                // for (auto i)
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

        bool is_root = generate_root();

        while (!list_open.empty() && !solution_found) {

            (void) update_list_focal();
            if (score_min >= cost_upperbound) {
                solution_cost = score_min;
                solution_found = false;
                break;
            }

            runtime = (double) (std::clock() - time_start) / CLOCKS_PER_SEC;
            if (runtime > time_limit || n_hl_expanded > node_limit) {
                solution_cost = -1;
                solution_found = false;
                break;
            }

            auto curr = list_focal.top();
            list_focal.pop();
            list_open.erase(curr->handle_open);

            // curr->paths
            std::cout << curr->conflicts_unknown.size() << std::endl;

            if (curr->conflicts_unknown.size() + curr->conflicts_known.size() == 0) {
                solution_found = true;
                solution_cost = curr->g_cummulative;
                goal_node = curr;
                break;
            }

            // some heuristic logic that i am temporarilly omitting

            n_hl_expanded += 1;
            curr->time_expanded = n_hl_expanded;
            bool found_bypass = true;
            while (found_bypass) {
                if (curr->conflicts_unknown.size() + curr->conflicts_known.size() == 0) {
                    solution_found = true;
                    solution_cost = curr->g_cummulative;
                    goal_node = curr;
                    break;
                }
                found_bypass = false;

                std::shared_ptr<CBSNode> children[2] = { std::make_shared<CBSNode>(), std::make_shared<CBSNode>() };

                curr->conflict_chosen = choose_conflict(curr);

                if (disjoint_splitting && curr->conflict_chosen->type == conflict_type::STANDARD) {
                    int first = (bool) (std::rand() % 2);
                    std::cout << "STANDARD CONFLICT IN DISJOINT SPLITTING" << std::endl;
                    
                    // if (first) {
                    //     children[0]->constraints = curr->conflict_chosen->constraint1;
                    //     int i1, i2;
                    //     double t1, t2, tX;
                    //     constraint_type type;
                    //     std::tie(i1, i2, t1, t2, tX, type) = curr->conflict_chosen->constraint1.back();
                    // }
                } else {
                    children[0]->constraints = curr->conflict_chosen->constraint1;
                    children[1]->constraints = curr->conflict_chosen->constraint2;

                    // TODO : add corridor and rectangle logic
                    // if (curr->conflict->type == conflict_type::RECTANGLE && rectangle_helper.strategy == rectangle_strategy::DR)
                    // {
                    //     int i = (bool)(rand() % 2);
                    //     for (const auto constraint : child[1 - i]->constraints)
                    //     {
                    //         child[i]->constraints.emplace_back(get<0>(constraint), get<1>(constraint), get<2>(constraint), get<3>(constraint), 
                    //                                                                             constraint_type::POSITIVE_BARRIER);
                    //     }
                    // }
                    // else if (curr->conflict->type == conflict_type::CORRIDOR && corridor_helper.getStrategy() == corridor_strategy::DC)
                    // {
                    //     int i = (bool)(rand() % 2);
                    //     assert(child[1 - i]->constraints.size() == 1);
                    //     auto constraint = child[1 - i]->constraints.front();
                    //     child[i]->constraints.emplace_back(get<0>(constraint), get<1>(constraint), get<2>(constraint), get<3>(constraint),
                    //         constraint_type::POSITIVE_RANGE);
                    // }
                }

                bool solved[2] = { false, false };
                std::vector<std::vector<PathElement>> copy(paths);

                for (auto i = 0; i < 2; ++i) {
                    if (i > 0) { paths = copy; }
                    solved[i] = generate_child(children[i], curr);
                }
            }

            

        // for (const auto &path : paths) {
        //     std::cout << "g : " << path->back().cost << ", h : " << path->back().heuristic << std::endl;
        }




        solution_found = true;
        return solution_found;
    }


    std::shared_ptr<Conflict> CBS::choose_conflict(const std::shared_ptr<CBSNode> &_node) const
    {
        std::shared_ptr<Conflict> conflict_chosen;
        if (_node->conflicts_known.empty() && _node->conflicts_unknown.empty()) {
            return nullptr;
        } else if (!_node->conflicts_known.empty()) {
            conflict_chosen = _node->conflicts_known.back();
            for (const auto &conflict : _node->conflicts_known) {
                if (*conflict_chosen < *conflict) {
                    conflict_chosen = conflict;
                }
            }
        } else {
            conflict_chosen = _node->conflicts_unknown.back();
            for (const auto &conflict : _node->conflicts_unknown) {
                if (*conflict_chosen < *conflict) {
                    conflict_chosen = conflict;
                }
            }
        }
        return conflict_chosen;
    }


    inline void CBS::update_paths(const std::shared_ptr<CBSNode> &_node)
    {
        for (auto i = 0; i < instance->n_actors; ++i) {
            paths[i] = paths_found_initially[i];
        }
        auto curr = _node;
        std::vector<bool> updated(instance->n_actors, false);
        while (curr != nullptr) {
            for (const auto &path : curr->paths) {
                if (!updated[path.first]) {
                    paths[path.first] = path.second;
                    updated[path.first] = true;
                }
            }
            curr = curr->parent;
        }
    }


    void CBS::save_paths(const std::string &_path_goals, bool _simplify)
    {
        for (auto i = 0; i < instance->n_actors; ++i) {
            auto path = paths[i];

            std::stringstream ss;
            ss << i + 1;
            std::string index_actor = ss.str();

            std::vector<std::shared_ptr<Vertex>> vertices;
            std::vector<std::tuple<double, double, double, double>> data;

            auto n_path = static_cast<int>(path.size());
            for (auto j = 0; j < n_path; ++j) {
                vertices.push_back(instance->graph->lookup_vertex(path[j].index_vertex));
            }

            if (!_simplify) {
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
