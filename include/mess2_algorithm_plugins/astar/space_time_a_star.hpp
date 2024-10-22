#ifndef MESS2_ALGORITHM_PLUGINS_SPACE_TIME_A_STAR_HPP
#define MESS2_ALGORITHM_PLUGINS_SPACE_TIME_A_STAR_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/astar/single_actor_solver.hpp"

namespace mess2_algorithms
{
    /**
     * 
     */
    class AStarNode : public LLNode
    {
    public:
        /**
         * 
         */
        typedef boost::heap::pairing_heap<std::shared_ptr<AStarNode>, boost::heap::compare<LLNode::compare_node_primary>>::handle_type open_handle_t;


        /**
         * 
         */
        typedef boost::heap::pairing_heap<std::shared_ptr<AStarNode>, boost::heap::compare<LLNode::compare_node_secondary>>::handle_type focal_handle_t;


        open_handle_t handle_open;
        focal_handle_t handle_focal;
        std::list<int> unsatisfied_positive_constraint_sets; // indices of satisfied positive constraint sets


        /**
         * 
         */
        AStarNode(const double _g, const double _h, const double _t, const int _index_node, const std::shared_ptr<LLNode> &_parent, const std::shared_ptr<Edge> &_edge_prev, bool _is_in_openlist = false) : LLNode(_g, _h, _t, _index_node, _parent, _edge_prev, _is_in_openlist) {};

        /**
         * 
         */
        ~AStarNode() {};


        /**
         * 
         */
        struct Hasher
        {
            size_t operator() (const std::shared_ptr<AStarNode> &_node) const
            {
                size_t hash_loc = std::hash<int>() (_node->edge_prev->vertex_child->point->index_point);
                size_t hash_t = std::hash<double>() (_node->t_curr);
                return hash_loc ^ (hash_t << 1);
            }
        };


        /**
         * 
         */
        struct Equality
        {
            bool operator() (const std::shared_ptr<AStarNode> &_node1, const std::shared_ptr<AStarNode> &_node2) const
            {
                return (_node1->edge_prev->vertex_child->point->index_point == _node2->edge_prev->vertex_child->point->index_point && _node1->wait_at_target == _node2->wait_at_target && _node1->unsatisfied_positive_constraint_sets == _node2->unsatisfied_positive_constraint_sets);
            }
        };
    };


    /**
     * 
     */
    class SpaceTimeAStar : public SingleActorSolver
    {
    public:
        /**
         * 
         */
        SpaceTimeAStar(std::shared_ptr<Instance> &_instance, int _index_actor) : SingleActorSolver(_instance, _index_actor) {};


        /**
         * 
         */
        Path find_path(std::shared_ptr<CBSNode> &_node, const ConstraintTable &_constraints_init, const std::vector<std::shared_ptr<Path>> &_paths, double _lowerbound) override;


        /**
         * 
         */
        Path retrieve_path(std::shared_ptr<AStarNode> _node);

    private:
        /**
         * 
         */
        typedef boost::heap::pairing_heap<std::shared_ptr<AStarNode>, boost::heap::compare<AStarNode::compare_node_primary>> heap_open_t;


        /**
         * 
         */
        typedef boost::heap::pairing_heap<std::shared_ptr<AStarNode>, boost::heap::compare<AStarNode::compare_node_secondary>> heap_focal_t;


        heap_open_t list_open;
        heap_focal_t list_focal;
        double score_min; // minimum score value in open
        double lowerbound; // threshold for focal


        /**
         * 
         */
        typedef boost::unordered_set<std::shared_ptr<AStarNode>, AStarNode::Hasher, AStarNode::Equality> hashtable_t;


        hashtable_t table_of_all_nodes;


        /**
         * 
         */
        Path find_path_shortest(ConstraintTable &_constraint_table, double _lowerbound);


        /**
         * 
         */
        void update_list_focal();


        /**
         * 
         */
        inline std::shared_ptr<AStarNode> pop_node();


        /**
         * 
         */
        inline void push_node(std::shared_ptr<AStarNode> &_node);


        /**
         * 
         */
        void release_nodes();


        friend class CBS; //
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_SPACE_TIME_A_STAR_HPP
