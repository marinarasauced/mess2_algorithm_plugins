#ifndef MESS2_ALGORITHM_PLUGINS_CBS_NODE_HPP
#define MESS2_ALGORITHM_PLUGINS_CBS_NODE_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/actor.hpp"
#include "mess2_algorithm_plugins/conflict.hpp"

namespace mess2_algorithms
{
    /**
     * enum representing different node selection strategies for CBSNode.
     */
    enum node_selection { RANDOM, H, DEPTH, CONFLICTS, CONFLICT_PAIRS, MVC };


    /**
     * @brief represents a node in the CBS (Conflict-Based Search) algorithm, with various comparison operators, path, constraint management, and heuristic properties.
     */
    class CBSNode
    {
    public:
        /**
         * @brief comparator for primary comparison of CBSNodes based on cumulative cost (g + h).
         */
        struct compare_node_primary
        {
            bool operator() (const std::shared_ptr<CBSNode> &_node1, const std::shared_ptr<CBSNode> &_node2) const
            {
                return (_node1->g_cummulative + _node1->h_cummulative >= _node2->g_cummulative + _node2->h_cummulative);
            }
        };


        /**
         * @brief comparator for secondary comparison of CBSNodes with a tiebreaker mechanism.
         */
        struct compare_node_secondary
        {
            bool operator() (const std::shared_ptr<CBSNode> &_node1, const std::shared_ptr<CBSNode> &_node2) const
            {
                if (_node1->tiebreak == _node2->tiebreak) {
                    return std::rand() % 2;
                }
                return (_node1->tiebreak >= _node2->tiebreak);
            }
        };


        /**
         * @brief handle type for the pairing heap used to store nodes in the open list (primary comparison).
         */
        typedef boost::heap::pairing_heap<std::shared_ptr<CBSNode>, boost::heap::compare<CBSNode::compare_node_primary>>::handle_type open_handle_t;


        /**
         * @brief handle type for the pairing heap used to store nodes in the focal list (secondary comparison).
         */
        typedef boost::heap::pairing_heap<std::shared_ptr<CBSNode>, boost::heap::compare<CBSNode::compare_node_secondary>>::handle_type focal_handle_t;


        open_handle_t handle_open;
        focal_handle_t handle_focal;


        /**
         * @brief hash function object for CBSNode, using the node's time_generated as the key.
         */
        struct Hasher
        {
            std::size_t operator() (const std::shared_ptr<CBSNode> _node)
            {
                return std::hash<double>() (_node->time_generated);
            }
        };


        std::list<std::shared_ptr<Conflict>> conflicts_known; // list of known conflicts
        std::list<std::shared_ptr<Conflict>> conflicts_unknown; // list of unknown conflicts
        std::shared_ptr<Conflict> conflict_chosen; // the chosen conflict
        std::shared_ptr<CBSNode> parent = nullptr;

        std::list<std::pair<int, Path>> paths; // new paths
        std::list<Constraint> constraints; // new constraints

        double g_cummulative;
        double h_cummulative;
        // double depth;
        size_t makespan = 0; // makespan over all paths
        double tiebreak = 0.0;
        bool is_h_computed = false;

        double time_expanded;
        double time_generated;


        /**
         * @brief cears the current node's conflicts and resets state.
         */
        void clear();
    };

    /**
     * @brief a hashing utility to compare and hash single-agent constraints for use in constraint-based search algorithms.
     */
    struct ConstraintsHasherSingle
    {
        int index_actor{};
        const std::shared_ptr<CBSNode> node{};
        ConstraintsHasherSingle(std::shared_ptr<CBSNode> _node, int &_index_actor) : index_actor(_index_actor), node(_node) {};


        struct Equality
        {
            bool operator() (const ConstraintsHasherSingle &_c1, const ConstraintsHasherSingle &_c2)
            {
                if (_c1.index_actor != _c2.index_actor) {
                    return false; // must have same actors to be equal
                }
                std::set<Constraint> cons1, cons2;
                std::shared_ptr<CBSNode> curr;
                
                curr = _c1.node;
                while (curr->parent != nullptr)
                {
                    for (const auto &constraint : curr->constraints)
                    {
                        auto index_actor = std::get<0>(constraint);
                        auto type_constraint = std::get<5>(constraint);

                        if (type_constraint == constraint_type::LEQLENGTH || type_constraint == constraint_type::POSITIVE_POINT || type_constraint == constraint_type::POSITIVE_VERTEX || type_constraint == constraint_type::POSITIVE_EDGE || index_actor == _c1.index_actor) {
                            cons1.insert(constraint);
                        }
                    }
                    curr = curr->parent;
                }

                curr = _c2.node;
                while (curr->parent != nullptr)
                {
                    for (const auto &constraint : curr->constraints)
                    {
                        auto index_actor = std::get<0>(constraint);
                        auto type_constraint = std::get<5>(constraint);

                        if (type_constraint == constraint_type::LEQLENGTH || type_constraint == constraint_type::POSITIVE_POINT || type_constraint == constraint_type::POSITIVE_VERTEX || type_constraint == constraint_type::POSITIVE_EDGE || index_actor == _c2.index_actor) {
                            cons2.insert(constraint);
                        }
                    }
                    curr = curr->parent;
                }

                return std::equal(cons1.begin(), cons1.end(), cons2.begin(), cons2.end());
            }
        };

        struct Hasher
        {
            std::size_t operator() (const ConstraintsHasherSingle &_c1)
            {
                std::shared_ptr<CBSNode> curr = _c1.node;
                size_t cons_hash = 0;
                while (curr->parent != nullptr)
                {
                    auto index_actor = std::get<0>(curr->constraints.front());
                    auto type_constraint = std::get<5>(curr->constraints.front());
                    if (type_constraint == constraint_type::LEQLENGTH || type_constraint == constraint_type::POSITIVE_POINT || type_constraint == constraint_type::POSITIVE_VERTEX || type_constraint == constraint_type::POSITIVE_EDGE || index_actor == _c1.index_actor) {
                        for (auto constraint : curr->constraints)
                        {
                            cons_hash += 3 * std::hash<int>() (std::get<0>(constraint)) + 5 * std::hash<int>() (std::get<1>(constraint)) + 7 * std::hash<double>() (std::get<2>(constraint)) + 11 * std::hash<double>() (std::get<3>(constraint));
                        }
                    }
                    curr = curr->parent;
                }
                return cons_hash;
            }
        };
    };

    /**
     * @brief a hashing utility to compare and hash multi-agent constraints for use in constraint-based search algorithms.
     */
    struct ConstraintsHasherDouble
    {
        int index_actor1{};
        int index_actor2{};
        const std::shared_ptr<CBSNode> node{};
        ConstraintsHasherDouble(std::shared_ptr<CBSNode> _node, int &_index_actor1, int &_index_actor2) : index_actor1(_index_actor1), index_actor2(_index_actor2), node(_node) {};


        struct Equality
        {
            bool operator() (const ConstraintsHasherDouble &_c1, const ConstraintsHasherDouble &_c2)
            {
                if (_c1.index_actor1 != _c2.index_actor1 || _c1.index_actor2 != _c2.index_actor2) {
                    return false; // must have same actors to be equal
                }
                std::set<Constraint> cons1[2], cons2[2];
                std::shared_ptr<CBSNode> curr;
                
                curr = _c1.node;
                while (curr->parent != nullptr)
                {
                    for (const auto &constraint : curr->constraints)
                    {
                        auto index_actor = std::get<0>(constraint);
                        auto type_constraint = std::get<5>(constraint);

                        if (type_constraint == constraint_type::LEQLENGTH || type_constraint == constraint_type::POSITIVE_POINT || type_constraint == constraint_type::POSITIVE_VERTEX || type_constraint == constraint_type::POSITIVE_EDGE) {
                            cons1[0].insert(constraint);
                            cons2[0].insert(constraint);
                        } else if (index_actor == _c1.index_actor1) {
                            cons1[0].insert(constraint);
                        } else if (index_actor == _c1.index_actor2) {
                            cons2[0].insert(constraint);
                        }
                    }
                    curr = curr->parent;
                }

                curr = _c2.node;
                while (curr->parent != nullptr)
                {
                    for (const auto &constraint : curr->constraints)
                    {
                        auto index_actor = std::get<0>(constraint);
                        auto type_constraint = std::get<5>(constraint);

                        if (type_constraint == constraint_type::LEQLENGTH || type_constraint == constraint_type::POSITIVE_POINT || type_constraint == constraint_type::POSITIVE_VERTEX || type_constraint == constraint_type::POSITIVE_EDGE) {
                            cons1[1].insert(constraint);
                            cons2[1].insert(constraint);
                        } else if (index_actor == _c2.index_actor1) {
                            cons1[1].insert(constraint);
                        } else if (index_actor == _c2.index_actor2) {
                            cons2[1].insert(constraint);
                        }
                    }
                    curr = curr->parent;
                }

                if (cons1[0].size() != cons1[1].size() || cons2[0].size() != cons2[1].size()) {
                    return false;
                }

                if (!std::equal(cons1[0].begin(), cons1[0].end(), cons1[1].begin())) {
                    return false;
                }

                return std::equal(cons2[0].begin(), cons2[0].end(), cons2[1].begin());
            }
        };

        struct Hasher
        {
            std::size_t operator() (const ConstraintsHasherDouble &_c1)
            {
                std::shared_ptr<CBSNode> curr = _c1.node;
                size_t cons1_hash, cons2_hash = 0;
                while (curr->parent != nullptr)
                {
                    for (auto constraint : curr->constraints)
                    {
                        auto index_actor = std::get<0>(constraint);
                        auto type_constraint = std::get<5>(constraint);

                        if (index_actor == _c1.index_actor1) {
                            cons1_hash += 3 * std::hash<int>() (std::get<0>(constraint)) + 5 * std::hash<int>() (std::get<1>(constraint)) + 7 * std::hash<double>() (std::get<2>(constraint)) + 11 * std::hash<double>() (std::get<3>(constraint));
                        } else if (index_actor == _c1.index_actor2) {
                            cons2_hash += 3 * std::hash<int>() (std::get<0>(constraint)) + 5 * std::hash<int>() (std::get<1>(constraint)) + 7 * std::hash<double>() (std::get<2>(constraint)) + 11 * std::hash<double>() (std::get<3>(constraint));
                        } else if (type_constraint == constraint_type::LEQLENGTH || type_constraint == constraint_type::POSITIVE_POINT || type_constraint == constraint_type::POSITIVE_VERTEX || type_constraint == constraint_type::POSITIVE_EDGE) {
                            cons1_hash += 3 * std::hash<int>() (std::get<0>(constraint)) + 5 * std::hash<int>() (std::get<1>(constraint)) + 7 * std::hash<double>() (std::get<2>(constraint)) + 11 * std::hash<double>() (std::get<3>(constraint));
                            cons2_hash += 3 * std::hash<int>() (std::get<0>(constraint)) + 5 * std::hash<int>() (std::get<1>(constraint)) + 7 * std::hash<double>() (std::get<2>(constraint)) + 11 * std::hash<double>() (std::get<3>(constraint));
                        }
                    }
                    curr = curr->parent;
                }
                return cons1_hash ^ (cons2_hash << 1);
            }
        };
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_CBS_NODE_HPP
