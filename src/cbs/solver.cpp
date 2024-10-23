
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
        // find_conflicts(*dummy_start)
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


    bool CBS::find_collisions(const std::pair<std::list<std::shared_ptr<Key3D>>, std::list<std::shared_ptr<Key3D>>> &_occupancies)
    {
        for (const auto &key1 : _occupancies.first) {
            for (const auto &key2 : _occupancies.second) {
                if (key1->index_key == key2->index_key) {
                    return true;
                }
            }
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
        auto sizes = std::pair<int, int>(static_cast<int>(path1.size()), static_cast<int>(path2.size()));

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
            if (t2 < t1) {
                vertex2 = instance->graph->lookup_vertex(path2[counter.second].index_vertex);
                key2 = vertex2->point->key;
                occupancies.second = actor2->lookup_occupancies_symbolically(instance->graph, key2->i, key2->j, key2->k, actor2->occupancies_symbolic);
                
                counter.second += 1;

            // update first actor
            } else {
                vertex1 = instance->graph->lookup_vertex(path1[counter.first].index_vertex);
                key1 = vertex1->point->key;
                occupancies.first = actor1->lookup_occupancies_symbolically(instance->graph, key1->i, key1->j, key1->k, actor1->occupancies_symbolic);
                counter.first += 1;
            }

            // determine if any collisions occur
            bool is_conflicting = find_collisions(occupancies);
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
                        if (*j = index_actor2) {
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
                for (auto index_actor2 = 0; index_actor2 < instance->n_actors; ++index_actor2) {
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





        return false;
    }

} // namespace mess2_algorithms
