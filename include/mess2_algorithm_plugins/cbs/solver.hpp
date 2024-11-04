#ifndef MESS2_ALGORITHM_PLUGINS_CBS_SOLVER_HPP
#define MESS2_ALGORITHM_PLUGINS_CBS_SOLVER_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/astar/space_time_a_star.hpp"

namespace mess2_algorithms
{
    class CBS
    {
    public:
        bool random_root = false; // randomize the order of actors in the root ct node
        bool use_cat = true;

        double runtime = 0.0;
        double runtime_generate_children = 0.0; // runtime of generating child nodes
        double runtime_build_ct = 0.0; // runtime of building constraint tables
        double runtime_build_cat = 0.0; // runtime of building conflict avoidance tables
        double runtime_path_finding = 0.0; // runtime of finding paths for single actors
        double runtime_detect_conflicts = 0.0; // runtime of detecting conflicts between actors
        double runtime_preprocessing = 0.0; // runtime of building (formlerly heuristic tables for low level)

        uint64_t n_conflicts_corridor = 0;
        uint64_t n_conflicts_rectangle = 0;
        uint64_t n_conflicts_target = 0;
        uint64_t n_conflicts_mutex = 0;
        uint64_t n_conflicts_standard = 0;

        uint64_t n_adopt_bypass = 0; // number of times when adopting bypasses

        uint64_t n_hl_expanded = 0;
        uint64_t n_hl_generated = 0;
        uint64_t n_ll_expanded = 0;
        uint64_t n_ll_generated = 0;

        std::shared_ptr<CBSNode> dummy_start = nullptr;
        std::shared_ptr<CBSNode> goal_node = nullptr;

        bool solution_found = false;
        double solution_cost = -2.0;
        double score_min;
        double threshold_list_focal;

        // /**
        //  * 
        //  */
        // void setHeuristicType(heuristics_type h) {heuristic_helper.type = h; }
        // void setPrioritizeConflicts(bool p) {PC = p;	heuristic_helper.PC = p; }
        // void setRectangleReasoning(rectangle_strategy r) {rectangle_helper.strategy = r; heuristic_helper.rectangle_reasoning = r; }
        // void setCorridorReasoning(corridor_strategy c) {corridor_helper.setStrategy(c); heuristic_helper.corridor_reasoning = c; }
        // void setTargetReasoning(bool t) {target_reasoning = t; heuristic_helper.target_reasoning = t; }
        // void setMutexReasoning(bool m) {mutex_reasoning = m; heuristic_helper.mutex_reasoning = m; }
        // void setDisjointSplitting(bool d) {disjoint_splitting = d; heuristic_helper.disjoint_splitting = d; }
        // void setBypass(bool b) { bypass = b; } // 2-actor solver for heuristic calculation does not need bypass strategy.
        // void setNodeLimit(int n) { node_limit = n; }
        // void setSavingStats(bool s) { save_stats = s; heuristic_helper.save_stats = s; }

        /**
         * 
         */
        bool solve(double _time_limit, double _cost_lowerbound = 0, double _cost_upperbound = MAX_COST);

        /**
         * 
         */
        CBS(std::shared_ptr<Instance> &_instance, bool _sipp, int _screen);


        /**
         * 
         */
        void save_paths(const std::string &_path_goals, bool _simplify);


    private:
        bool target_reasoning = false;
        bool disjoint_splitting = false;
        bool mutex_reasoning;
        bool bypass;
        bool PC;
        bool save_stats;

        // MDDTable mdd_helper;	
        // RectangleReasoning rectangle_helper;
        // CorridorReasoning corridor_helper;
        // MutexReasoning mutex_helper;
        // CBSHeuristic heuristic_helper;

        boost::heap::pairing_heap<std::shared_ptr<CBSNode>, boost::heap::compare<CBSNode::compare_node_primary>> list_open;
        boost::heap::pairing_heap<std::shared_ptr<CBSNode>, boost::heap::compare<CBSNode::compare_node_secondary>> list_focal;
        std::list<std::shared_ptr<CBSNode>> table_of_all_nodes;

        int screen;

        double time_limit = 10.0;
        int node_limit = MAX_NODES;
        double focal_w = 1.0;
        double cost_upperbound = MAX_COST;

        std::vector<ConstraintTable> initial_constraints;
        std::clock_t time_start;

        std::vector<Path> paths;
        std::vector<Path> paths_found_initially;
        std::vector<std::shared_ptr<SingleActorSolver>> search_engines; // used to find single actors' paths and mdd

        std::shared_ptr<Instance> &instance;


        /**
         * 
         */
        bool generate_root();


        /**
         * 
         */
        bool generate_child(std::shared_ptr<CBSNode> &_node, const std::shared_ptr<CBSNode> &_parent);


        /**
         * 
         */
        bool find_path_for_single_actor(std::shared_ptr<CBSNode> &_node, int _index_actor, double _lowerbound);


        /**
         * 
         */
        void copy_conflicts(const std::list<std::shared_ptr<Conflict>> &_conflicts, std::list<std::shared_ptr<Conflict>> &_copy, const std::list<int> &_actors_excluded);


        /**
         * 
         */
        std::shared_ptr<Conflict> choose_conflict(const std::shared_ptr<CBSNode> &_node) const;


        /**
         * 
         */
        bool find_collisions(const std::shared_ptr<Key3D> &_key1, const std::shared_ptr<Key3D> &_key2, const double &_radius);


        /**
         * 
         */
        void find_conflicts(std::shared_ptr<CBSNode>& _node);


        /**
         * 
         */
        void find_conflicts(std::shared_ptr<CBSNode>& _node, int &_index_actor1, int &_index_actor2);


        /**
         * 
         */
        void update_list_focal();


        /**
         * 
         */
        inline void update_paths(const std::shared_ptr<CBSNode> &_node);
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_CBS_SOLVER_HPP
