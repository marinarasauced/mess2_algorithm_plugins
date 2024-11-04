
#include "mess2_algorithm_plugins/constraint_table.hpp"

namespace mess2_algorithms
{
    void ConstraintTable::insert_to_ct(int &_index_point, double &_t1, double &_t2)
    {
        assert(_index_point >= 0 && _index_point < instance->graph->n_points);
        ct[_index_point].emplace_back(_t1, _t2);

        if (_t2 < MAX_TIMESTEP && _t2 > time_latest) {
            time_latest = _t2;
        } else if (_t2 == MAX_TIMESTEP && _t1 > time_latest) {
            time_latest = _t1;
        }
    }

    void ConstraintTable::insert_landmark(int _index_point, double _t)
    {
        auto iterator = landmarks.find(_index_point);
        if (iterator == landmarks.end()) {
            landmarks[_index_point].push_back(_t);
            if (_t > time_latest) {
                time_latest = _t;
            }
        } else {
            assert(std::find(iterator->second.begin(), iterator->second.end(), _t) != iterator->second.end());
        }
    }

    void ConstraintTable::copy(const ConstraintTable &_other)
    {
        time_min = _other.time_min;
        time_max = _other.time_max;
        time_latest = _other.time_latest;
        ct = _other.ct;
        landmarks = _other.landmarks;
        instance = _other.instance;
    }

    void ConstraintTable::build_ct(int &_index_actor, const std::shared_ptr<CBSNode> &_node)
    {
        auto curr = _node;
        std::cout << "ConstraintTable::build_ct : starting build" << std::endl;
        while (curr->parent != nullptr)
        {
            int i1, i2;
            double t1, t2, tX;
            constraint_type type;
            std::tie(i1, i2, t1, t2, tX, type) = curr->constraints.front();
            std::shared_ptr<Key3D> _key1;
            std::shared_ptr<Key3D> _key2;
            int index_point1 = -1;
            int index_point2 = -1;
            switch(type)
            {
            case constraint_type::LEQLENGTH:
                assert (curr->constraints.size() <= 2);
                if (_index_actor == i1) {
                    time_max = std::min(time_max, t2);
                } else {
                    auto key = instance->graph->lookup_key(i2);
                    auto index_point = instance->graph->find_index_point(key->i, key->j, key->k);
                    (void) insert_to_ct(index_point, t1, t2);
                }
                // if ((int)curr->constraints.size() == 2) // generated by a corridor-target conflict
                // {
                //     tie(a, x, y, t, type) = curr->constraints.back();
                //     if (type == constraint_type::GLENGTH && a == actor)
                //         length_min = max(length_min, t + 1); // path of this actor should be of length at least t + 1
                //     else if (type == constraint_type::RANGE && a == actor)
                //     {
                //         insert2CT(x, y, t + 1); // the actor cannot stay at x from timestep y to timestep t.
                //     }
                //     else
                //     {
                //         assert(1); // this should never happen
                //     }
                // }
                break;
            case constraint_type::GLENGTH:
                if (_index_actor == i1) {
                    time_min = std::max(time_min, t2);
                }
                break;
            case constraint_type::POSITIVE_POINT:
                _key1 = instance->graph->lookup_key(i2);
                index_point1 = instance->graph->find_index_point(_key1->i, _key1->j, _key1->k);
                if (_index_actor == i1) {
                    (void) insert_landmark(index_point1, tX);
                } else {
                    auto radius = instance->actors[_index_actor]->radius + instance->actors[i1]->radius; // sum of radii of actors in constraint generation
                    auto occupancies_symbolic = compute_occupancies_symbolically(instance->graph, radius);

                    auto keys = instance->actors[_index_actor]->lookup_occupancies_symbolically(instance->graph, _key1->i, _key1->j, _key1->k, occupancies_symbolic);

                    for (auto key : keys) {
                        auto index_point = instance->graph->find_index_point(key->i, key->j, key->k);
                        (void) insert_to_ct(index_point, t1, t2);
                    }
                }
                break;
            case constraint_type::POSITIVE_VERTEX:
                _key1 = instance->graph->lookup_vertex(i2)->point->key;
                index_point1 = instance->graph->find_index_point(_key1->i, _key1->j, _key1->k);
                if (_index_actor == i1) {
                    (void) insert_landmark(index_point1, tX);
                } else {
                    auto radius = instance->actors[_index_actor]->radius + instance->actors[i1]->radius; // sum of radii of actors in constraint generation
                    auto occupancies_symbolic = compute_occupancies_symbolically(instance->graph, radius);

                    auto keys = instance->actors[_index_actor]->lookup_occupancies_symbolically(instance->graph, _key1->i, _key1->j, _key1->k, occupancies_symbolic);

                    for (auto key : keys) {
                        auto index_point = instance->graph->find_index_point(key->i, key->j, key->k);
                        (void) insert_to_ct(index_point, t1, t2);
                    }
                }
                break;
            case constraint_type::POSITIVE_EDGE:
                _key1 = instance->graph->lookup_edge(i2)->vertex_parent->point->key;
                _key2 = instance->graph->lookup_edge(i2)->vertex_child->point->key;
                index_point1 = instance->graph->find_index_point(_key1->i, _key1->j, _key1->k);
                index_point2 = instance->graph->find_index_point(_key2->i, _key2->j, _key2->k);                
                if (_index_actor == i1) {
                    (void) insert_landmark(index_point1, t1);
                    (void) insert_landmark(index_point2, t2);
                } else {
                    auto radius = instance->actors[_index_actor]->radius + instance->actors[i1]->radius; // sum of radii of actors in constraint generation
                    auto occupancies_symbolic = compute_occupancies_symbolically(instance->graph, radius);

                    auto keys1 = instance->actors[_index_actor]->lookup_occupancies_symbolically(instance->graph, _key1->i, _key1->j, _key1->k, occupancies_symbolic);
                    auto keys2 = instance->actors[_index_actor]->lookup_occupancies_symbolically(instance->graph, _key2->i, _key2->j, _key2->k, occupancies_symbolic);

                    auto _t1 = std::max(0.0, t1 - tX);
                    auto _t2 = std::min(MAX_TIMESTEP, t2 + tX);

                    for (auto key : keys1) {
                        auto index_point = instance->graph->find_index_point(key->i, key->j, key->k);
                        (void) insert_to_ct(index_point, _t1, t2);
                    }

                    for (auto key : keys2) {
                        auto index_point = instance->graph->find_index_point(key->i, key->j, key->k);
                        (void) insert_to_ct(index_point, t1, _t2);
                    }
                }
                break;
            case constraint_type::POINT:
                if (_index_actor == i1) {
                    std::cout << t1 << ", " << t2 << std::endl;
                    _key1 = instance->graph->lookup_key(i2);
                    auto occupancies_symbolic = compute_occupancies_symbolically(instance->graph, tX);
                    auto keys = instance->actors[i1]->lookup_occupancies_symbolically(instance->graph, _key1->i, _key1->j, _key1->k, occupancies_symbolic);

                    for (auto key : keys) {
                        (void) insert_to_ct(key->index_key, t1, t2);
                    }
                }
                break;
            case constraint_type::VERTEX:
                if (_index_actor == i1) {
                    _key1 = instance->graph->lookup_vertex(i2)->point->key;
                    auto keys = instance->actors[i1]->lookup_occupancies_symbolically(instance->graph, _key1->i, _key1->j, _key1->k, instance->actors[i1]->occupancies_symbolic);

                    for (auto key : keys) {
                        auto index_point = instance->graph->find_index_point(key->i, key->j, key->k);
                        (void) insert_to_ct(index_point, t1, t2);
                    }
                }
                break;
            case constraint_type::EDGE:
                if (_index_actor == i1) {
                    _key1 = instance->graph->lookup_edge(i2)->vertex_parent->point->key;
                    _key2 = instance->graph->lookup_edge(i2)->vertex_child->point->key;

                    auto keys1 = instance->actors[i1]->lookup_occupancies_symbolically(instance->graph, _key1->i, _key1->j, _key1->k, instance->actors[i1]->occupancies_symbolic);
                    auto keys2 = instance->actors[i1]->lookup_occupancies_symbolically(instance->graph, _key2->i, _key2->j, _key2->k, instance->actors[i1]->occupancies_symbolic);

                    auto _t1 = std::max(0.0, t1 - tX);
                    auto _t2 = std::min(MAX_TIMESTEP, t2 + tX);

                    for (auto key : keys1) {
                        auto index_point = instance->graph->find_index_point(key->i, key->j, key->k);
                        (void) insert_to_ct(index_point, _t1, t2);
                    }

                    for (auto key : keys2) {
                        auto index_point = instance->graph->find_index_point(key->i, key->j, key->k);
                        (void) insert_to_ct(index_point, t1, _t2);
                    }
                }
                break;
            case constraint_type::BARRIER:
                break;
            case constraint_type::RANGE:
                break;
            case constraint_type::POSITIVE_BARRIER:
                break;
            case constraint_type::POSITIVE_RANGE:
                break;
            }
            curr = curr->parent;
        }
        if (time_latest < time_min) {
            time_latest = time_min;
        }
        if (time_max < MAX_TIMESTEP && time_latest < time_max) {
            time_latest = time_max;
        }
    }

    void ConstraintTable::build_cat(int &_index_actor, const std::vector<Path> &_paths)
    {
        if (time_min >= MAX_TIMESTEP || time_min > time_max) {
            return; // cannot reach goal so do not build cat
        }

        for (auto i1 = 0; i1 < instance->n_actors; ++i1) {
            if (_index_actor == i1 || _paths[i1].empty()) {
                continue;
            }

            auto radius = instance->actors[_index_actor]->radius + instance->actors[i1]->radius; // sum of radii of two actors in consideration
            auto occupancies_symbolic = compute_occupancies_symbolically(instance->graph, radius);

            auto path = _paths[i1];
            auto n_path = static_cast<int>(path.size());

            PathElement elem_prev;
            PathElement elem_curr;
            PathElement elem_next;

            for (int iter = 0; iter < n_path; ++iter)
            {
                if (iter != 0) {
                    elem_prev = elem_curr;
                } else {
                    elem_prev = {-1, 0.0, 0.0, 0.0};
                }

                elem_curr = path[iter];

                if (iter < n_path) {
                    elem_next = path[iter + 1];
                } else {
                    elem_next = {elem_curr.index_vertex, MAX_TIMESTEP, elem_curr.cost, elem_curr.heuristic};
                }

                auto t1 = elem_prev.time;
                auto t2 = elem_next.time;

                auto _key = instance->graph->lookup_vertex(elem_curr.index_vertex)->point->key;
                auto keys = instance->actors[_index_actor]->lookup_occupancies_symbolically(instance->graph, _key->i, _key->j, _key->k, occupancies_symbolic);

                for (const auto &key : keys) {
                    auto index_point = instance->graph->find_index_point(key->i, key->j, key->k);
                    cat[index_point].emplace_back(t1, t2);
                }
            }
        }
    }

    double ConstraintTable::lookup_time_hold(int &_index_point)
    {
        auto rst = time_min;
        auto iterator = ct.find(_index_point);
        if (iterator != ct.end()) {
            for (auto time_range : iterator->second) {
                rst = std::max(rst, time_range.second);
            }
        }
        for (auto landmark : landmarks) {
            if (landmark.first == _index_point) {
                continue;
            }

            for (auto times : landmark.second) {
                rst = std::max(rst, times);
            }
        }
        return rst;
    }

    bool ConstraintTable::is_constrained(int &_index_point, double _t) const
    {
        auto iterator1 = landmarks.find(_index_point);
        if (iterator1 != landmarks.end()) {
            if (std::find(iterator1->second.begin(), iterator1->second.end(), _t) != iterator1->second.end()) { return true; }
        }

        auto iterator2 = ct.find(_index_point);
        auto iterator3 = cat.find(_index_point);
        if (iterator2 == ct.end() && iterator3 == cat.end()) {
            return false;
        }

        if (iterator2 != ct.end()) {
            for (const auto &time_range : iterator2->second) {
                if (_t >= time_range.first && _t <= time_range.second) {
                    return true;
                }
            }
        }

        if (iterator3 != cat.end()) {
            for (const auto &time_range : iterator3->second) {
                if (_t >= time_range.first && _t <= time_range.second) {
                    return true;
                }
            }
        }

        return false;
    }

    bool ConstraintTable::update_unstatisfied_positive_constraint_set(const std::list<int> &_old, std::list<int> &_new, int &_index_point, double _t)
    {
        for (auto i : _old) {
            _new.push_back(i);
            if (positive_constraint_sets[i].front().second <= _t && _t <= positive_constraint_sets[i].back().second) {
                for (const auto &state : positive_constraint_sets[i]) {
                    if (state.second == _t && state.first == _index_point) {
                        _new.pop_back();
                        break;
                    }
                }
            } else if (positive_constraint_sets[i].back().second < _t) {
                return false;
            }
        }
        return true;
    }

} // namespace mess2_algorithms
