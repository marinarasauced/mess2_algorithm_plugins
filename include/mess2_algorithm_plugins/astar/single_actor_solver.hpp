#ifndef MESS2_ALGORITHM_PLUGINS_SINGLE_ACTOR_SOLVER_HPP
#define MESS2_ALGORITHM_PLUGINS_SINGLE_ACTOR_SOLVER_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/constraint_table.hpp"

namespace mess2_algorithms
{
    /**
     * 
     */
    struct LLNodeElement
    {
        double g_curr;
        double h_curr;
        double n_conflicts;
        int index_node;

        LLNodeElement(double _g_curr, double _h_curr, double _n_conflicts, int _index_node) : g_curr(_g_curr), h_curr(_h_curr), n_conflicts(_n_conflicts), index_node(_index_node) {};

        struct compare_node_primary
        {
            bool operator() (const LLNodeElement &_node1, const LLNodeElement &_node2) const
            {
                if (_node1.g_curr + _node1.h_curr == _node2.g_curr + _node2.h_curr) {
                    if (_node1.h_curr == _node2.h_curr) {
                        return std::rand() % 2;
                    }
                    return _node1.h_curr >= _node2.h_curr;
                }
                return _node1.g_curr + _node1.h_curr >= _node2.g_curr + _node2.h_curr;
            }
        };

        struct compare_node_secondary
        {
            bool operator() (const LLNodeElement &_node1, const LLNodeElement &_node2) const
            {
                if (_node1.n_conflicts == _node2.n_conflicts) {
                    if (_node1.g_curr == _node2.g_curr) {
                        return std::rand() % 2 == 0;
                    }
                    return _node1.g_curr <= _node2.g_curr; // break ties towards larger g's
                }
                return _node1.n_conflicts >= _node2.n_conflicts;
            }
        };
    };


    /**
     * 
     */
    class LLNode
    {
    public:
        double g_curr; // cummulative weight cost to get to current node
        double h_curr; // cummulative heuristic cost to get to current node
        double t_curr; // cummulative time of travel to get to current node
        double score; // cummulative score of current node
        int index_node; // generation index of the current LLNode instance

        std::shared_ptr<LLNode> parent; // shared pointer to parent node during expansion

        std::shared_ptr<Edge> edge_prev; // shared pointer to previous edge (edge taken to transition from source (prev) vertex to target (curr) vertex)

        int n_conflicts = 0; // number of conflicts;
        bool is_in_openlist = false;
        bool wait_at_target;


        /**
         * @brief constructs a low level node instance without pointers to a shared parent node instance and the previous edge.
         */
        LLNode(const double _g, const double _h, const double _t, const int _index_node) : g_curr(_g), h_curr(_h), t_curr(_t), index_node(_index_node), parent(nullptr), edge_prev(nullptr), is_in_openlist(false), wait_at_target(false) { score = g_curr + h_curr; };


        /**
         * @brief constructs a low level node instance with pointers to a shared parent node instance and the previous edge.
         */
        LLNode(const double _g, const double _h, const double _t, const int _index_node, const std::shared_ptr<LLNode> &_parent, const std::shared_ptr<Edge> &_edge_prev, bool _is_in_openlist = false) : g_curr(_g), h_curr(_h), t_curr(_t), index_node(_index_node), parent(_parent), edge_prev(_edge_prev), is_in_openlist(_is_in_openlist), wait_at_target(false) { score = g_curr + h_curr; };


        /**
         * 
         */
        struct compare_node_primary
        {
            bool operator() (const std::shared_ptr<LLNode> &_node1, const std::shared_ptr<LLNode> &_node2) const
            {
                if (_node1->score == _node2->score) {
                    if (_node1->h_curr == _node2->h_curr) {
                        return std::rand() % 2;
                    }
                    return _node1->h_curr >= _node2->h_curr;
                }
                return _node1->score >= _node2->score;
            }
        };


        /**
         * 
         */
        struct compare_node_secondary
        {
            bool operator() (const std::shared_ptr<LLNode> &_node1, const std::shared_ptr<LLNode> &_node2) const
            {
                if (_node1->n_conflicts == _node2->n_conflicts) {
                    if (_node1->g_curr == _node2->g_curr) {
                        return std::rand() % 2 == 0;
                    }
                    return _node1->g_curr <= _node2->g_curr; // break ties towards larger g's
                }
                return _node1->n_conflicts >= _node2->n_conflicts;
            }
        };


        /**
         * 
         */
        inline double lookup_score() const { return score; }


        /**
         * 
         */
        void copy (const LLNode &_other)
        {
            g_curr = _other.g_curr;
            h_curr = _other.h_curr;
            t_curr = _other.t_curr;
            score = _other.score;
            parent = _other.parent;
            edge_prev = _other.edge_prev;
            n_conflicts = _other.n_conflicts;
            wait_at_target = _other.wait_at_target;
        }
    };

    /**
     * 
     */
    class SingleActorSolver
    {
    public:
        uint64_t n_generated = 0; // number of LLNode instances generated
        uint64_t n_expanded = 0; // number of LLNode instaces expanded

        double runtime_build_ct = 0.0; // runtime of building constraint table
        double runtime_build_cat = 0.0; // runtime of building conflict avoidance table

        std::shared_ptr<Instance> instance; // shared pointer to Instance instance


        /**
         * 
         */
        SingleActorSolver(std::shared_ptr<Instance> &_instance, int _index_actor) : instance(_instance), index_actor(_index_actor), actor(instance->actors[index_actor]) {};


        /**
         * 
         */
        virtual ~SingleActorSolver() {};


        /**
         * 
         */
        virtual Path find_path(std::shared_ptr<CBSNode> &_node, const ConstraintTable &_constraints_init, const std::vector<std::shared_ptr<Path>> &_paths, double _lowerbound) = 0;


        friend class SpaceTimeAStar;
        friend class CBS;

        int index_actor;
        std::shared_ptr<Actor> actor;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_SINGLE_ACTOR_SOLVER_HPP
