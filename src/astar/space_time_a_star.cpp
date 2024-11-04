
#include "mess2_algorithm_plugins/astar/space_time_a_star.hpp"

namespace mess2_algorithms
{
    Path SpaceTimeAStar::find_path(std::shared_ptr<CBSNode> &_node, const ConstraintTable &_constraints_init, const std::vector<Path> &_paths, double _lowerbound, bool &_use_path_avoidance)
    {
        n_expanded = 0;
        n_generated = 0;

        auto tic = std::clock();
        ConstraintTable table_constraints(_constraints_init);
        (void) table_constraints.build_ct(index_actor, _node);
        runtime_build_ct = (double) (std::clock() - tic) / CLOCKS_PER_SEC;

        if (table_constraints.time_min >= MAX_TIMESTEP || table_constraints.time_min > table_constraints.time_max || table_constraints.is_constrained(instance->graph->lookup_vertex(actor->index_source)->point->index_point, 0.0)) {
            return Path(); // cannot reach target or cannot stay at source
        }

        tic = std::clock();
        if (_use_path_avoidance) {
            (void) table_constraints.build_cat(index_actor, _paths);
            runtime_build_cat = (double) (std::clock() - tic) / CLOCKS_PER_SEC;
        }

        if (table_constraints.get_n_landmarks() > 0) {
            return Path();
        } else {
            return find_path_shortest(table_constraints, _lowerbound);
        }
    }


    Path SpaceTimeAStar::find_path_shortest(ConstraintTable &_constraint_table, double _lowerbound)
    {
        Path path;

        auto vertex_target = instance->graph->lookup_vertex(actor->index_target);
        auto edge_start = instance->graph->find_edge_by_vertex_indices(actor->index_source, actor->index_source);
        auto start = std::make_shared<AStarNode>(0.0, 0.0, 0.0, 0, nullptr, edge_start);
        
        std::list<int> positive_constraint_sets; // positive constraint sets by index via key -> doubles, key -> doubles, etc.
        for (size_t iter = 0; iter < _constraint_table.get_n_landmarks(); ++iter) {
            positive_constraint_sets.push_back(iter);
        }

        bool keep = _constraint_table.update_unstatisfied_positive_constraint_set(positive_constraint_sets, start->unsatisfied_positive_constraint_sets, start->edge_prev->vertex_child->point->index_point, start->t_curr);
        if (!keep) {
            return path;
        }

        n_generated += 1;
        start->handle_open = list_open.push(start);
        start->handle_focal = list_focal.push(start);
        start->is_in_openlist = true;
        table_of_all_nodes.insert(start);
        score_min = (double) start->score;
        double time_hold = _constraint_table.lookup_time_hold(vertex_target->point->index_point);
        lowerbound = std::max(time_hold - start->t_curr, std::max(score_min, _lowerbound));

        auto visits = std::vector<int>(instance->graph->n_vertices, 0);

        while (!list_open.empty())
        // for (auto wter = 0; wter < 2000; ++wter)
        {
 
            update_list_focal();
            auto curr = pop_node();
            
            
            // // PLEASE CHANGE THIS JUST TESTING IF N VISITS SLOWS ALG DOWN A LOT
            if (visits[curr->edge_prev->vertex_child->index_vertex] > 200) {
                // std::cout << visits[curr->edge_prev->index_edge] << std::endl;
                continue;
            } else {
                visits[curr->edge_prev->vertex_child->index_vertex] += 1;
            }

            if (curr->edge_prev->vertex_child->index_vertex == actor->index_target && !curr->wait_at_target && curr->t_curr >= time_hold)
            {
                path = retrieve_path(curr);
                break;
            }

            if (curr->t_curr >= _constraint_table.time_max) {
                continue;
            }

            auto edges_possible = instance->graph->lookup_edges(curr->edge_prev->vertex_child->index_vertex, curr->edge_prev->type);
            for (const auto &edge : edges_possible)
            {
                auto dt = actor->lookup_t(edge->index_edge);
                auto t_next = curr->t_curr + dt;
                // static logic i need to reread from ref

                if (_constraint_table.is_constrained(edge->vertex_child->point->index_point, t_next) || actor->lookup_obstacle_avoidance_table(edge->vertex_child->point->index_point)) {
                    continue;
                }

                auto dg = actor->lookup_g(edge->vertex_child->point->index_point);
                auto g_next = curr->g_curr + dg * dt;

                auto dh = actor->lookup_h(edge->vertex_child->point->index_point);
                auto h_next = curr->h_curr + dh * dt;
                h_next = 0.0;
                // if score next is greater than allowable score, continue

                auto next = std::make_shared<AStarNode>(g_next, h_next, t_next, static_cast<int>(table_of_all_nodes.size()), curr, edge);

                if (edge->vertex_child->index_vertex == actor->index_target && curr->edge_prev->vertex_child->index_vertex ==actor->index_target) {
                    next->wait_at_target = true;
                }

                keep = _constraint_table.update_unstatisfied_positive_constraint_set(curr->unsatisfied_positive_constraint_sets, next->unsatisfied_positive_constraint_sets, edge->vertex_child->point->index_point, next->t_curr);
                if (!keep) {
                    next.reset();
                    continue;
                }

                auto iterator = table_of_all_nodes.find(next);
                if (iterator == table_of_all_nodes.end())
                {
                    push_node(next);
                    table_of_all_nodes.insert(next);
                    continue;
                }

                auto existing_next = *iterator;
                if (existing_next->score > next->score || (existing_next->score == next->score && existing_next->n_conflicts > next->n_conflicts))
                {
                    if (!existing_next->is_in_openlist) {
                        existing_next->copy(*next);
                        push_node(existing_next);
                    } else {
                        bool add_to_focal = false; // check if it was above the focal bound before and now below (thus need to be inserted)
                        bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
                        bool update_open = false;

                        if ((g_next + h_next) <= lowerbound) {
                            // if the new f-val qualify to be in FOCAL
                            if (existing_next->score > lowerbound) { 
                                add_to_focal = true; // and the previous f-val did not qualify to be in FOCAL then add
                            } else {
                                update_in_focal = true; // and the previous f-val did qualify to be in FOCAL then update
                            }
                        }
                        if (existing_next->score > g_next + h_next) {
                            update_open = true;
                        }

                        existing_next->copy(*next); // update existing node

                        if (update_open) {
                            list_open.increase(existing_next->handle_open); // increase bc score improved
                        }
                        if (add_to_focal) {
                            existing_next->handle_focal = list_focal.push(existing_next);
                        }
                        if (update_in_focal) {
                            list_focal.update(existing_next->handle_focal);
                        }
                    }
                }
                next.reset();
            }
        }
        (void) release_nodes();
        // int visits_max = 0;
        // for (const auto &visit : visits) {
        //     if (visit > visits_max) {
        //         visits_max = visit;
        //     }
        // }
        // std::cout << visits_max << std::endl;
        return path;
    }


    void SpaceTimeAStar::update_list_focal()
    {
        auto head_open = list_open.top();
        if (head_open->score > score_min)
        {
            double score_min_new = (double) head_open->score;
            double lowerbound_new = std::max(lowerbound, score_min_new);
            for (auto n : list_open)
            {
                if (n->score > lowerbound && n->score <= lowerbound_new) {
                    n->handle_focal = list_focal.push(n);
                }
            }
            score_min = score_min_new;
            lowerbound = lowerbound_new;
        }
    }


    inline std::shared_ptr<AStarNode> SpaceTimeAStar::pop_node()
    {
        auto node = list_focal.top();
        list_focal.pop();
        list_open.erase(node->handle_open);
        node->is_in_openlist = false;
        n_expanded += 1;
        return node;
    }


    inline void SpaceTimeAStar::push_node(std::shared_ptr<AStarNode> &_node)
    {
        _node->handle_open = list_open.push(_node);
        _node->is_in_openlist = true;
        n_generated += 1;
        if (_node->score <= lowerbound) {
            _node->handle_focal = list_focal.push(_node);
        }
    }


    Path SpaceTimeAStar::retrieve_path(std::shared_ptr<AStarNode> _node)
    {
        Path path;

        std::shared_ptr<LLNode> curr = _node;
        while (curr != nullptr)
        {
            PathElement elem = {curr->edge_prev->vertex_child->index_vertex, curr->t_curr, curr->g_curr, curr->h_curr};
            path.push_back(elem);

            curr = curr->parent;
        }

        if (curr != nullptr) {
            PathElement elem_init = {curr->edge_prev->vertex_child->index_vertex, curr->t_curr, curr->g_curr, curr->h_curr};
            path.push_back(elem_init);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }


    void SpaceTimeAStar::release_nodes()
    {
        list_open.clear();
        list_focal.clear();
        for (auto node : table_of_all_nodes) {
            node.reset();
        }
        table_of_all_nodes.clear();
    }

} // namespace mess2_algorithms
