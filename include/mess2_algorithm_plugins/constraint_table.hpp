#ifndef MESS2_ALGORITHM_PLUGINS_CONSTRAINT_TABLE_HPP
#define MESS2_ALGORITHM_PLUGINS_CONSTRAINT_TABLE_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/cbs/node.hpp"
#include "mess2_algorithm_plugins/instance.hpp"

namespace mess2_algorithms
{
    /**
     * 
     */
    class ConstraintTable
    {
    public:
        double time_min = 0.0; // minimum time actor must navigate for
        double time_max = MAX_TIMESTEP; // maximum time actor can navigate
        double time_latest = 0.0; // no negative constraints after time_latest


        /**
         * 
         */
        ConstraintTable(const std::shared_ptr<Instance> &_instance) : instance(_instance) {};


        /**
         * 
         */
        ConstraintTable(const ConstraintTable &_other) { (void) copy(_other); };


        /**
         * @brief looks up the first time an actor can hold a point.
         * 
         * @param _index_point the index of the point being looked up.
         * @return the first time the point can be held.
         */
        double lookup_time_hold(int &_index_point);


        /**
         * @brief determines if a point is constraint at a time instance.
         * 
         * @param _index_point the index of the point in question.
         * @param _t the time at which the point is to be checked for constraints.
         * @return true if the point is constrained, false otherwise.
         */
        bool is_constrained(int &_index_point, double _t) const;


        /**
         * @brief copies another ConstraintTable instance's attributes to the current instance.
         * 
         * @param _other the other ConstraintTable instance whose attributes are to be copied.
         */
        void copy(const ConstraintTable &_other);


        /**
         * @brief builds the constraint table for a given actor and a given conflict based search node.
         * 
         * @param _index_actor the index of the actor to build the constraint table about.
         * @param _node the shared CBSNode pointer to build the constraint table about.
         */
        void build_ct(int &_index_actor, const std::shared_ptr<CBSNode> &_node);


        /**
         * @brief builds the conflict avoidance table for a given actor assuming knowledge of the previous actor's paths.
         */
        void build_cat(int &_index_actor, const std::vector<Path> &_paths);


        /**
         * @brief inserts a time constraint to the constraint table.
         * 
         * @param _index_point the index of the point to be constrained.
         * @param _t1 the lowerbound of the time constraint on the point.
         * @param _t2 the upperbound of the time constraint on the point.
         */
        void insert_to_ct(int &_index_point, double &_t1, double &_t2); // insert a 


        /**
         * 
         */
        void clear_ct() {
            ct.clear();
        }


        /**
         * @brief gets the number of landmarks, where a landmark is defined as a location with a time.
         */
        size_t get_n_landmarks()
        {
            size_t n_landmarks = 0;
            for (const auto &pair : landmarks) {
                n_landmarks += pair.second.size();
            }
            return n_landmarks;
        }


        /**
         * @brief updates the unsatisfied positive constraint set based on the old and new lists, the given point index, and time.
         * 
         * @param _old the old unsatisfied positive constraint set.
         * @param _new the new unsatisfied positive constraint set.
         * @param _index_point
         * @param _t
         * @return
         */
        bool update_unstatisfied_positive_constraint_set(const std::list<int> &_old, std::list<int> &_new, int &_index_point, double _t);


    protected:
        std::unordered_map<int, std::list<std::pair<double, double>>> ct; // constraint table by key, constraints at highest level prevent movement to a point

        std::unordered_map<int, std::vector<double>> landmarks; // locations (via index_point) that an actor nust be at at some time instance

        std::vector<std::list<std::pair<int, double>>> positive_constraint_sets; // vector of positive constraints set, each of which is a sorted list of Key3D, time instance


        /**
         * @brief inserts a landmark; i.e., the current actor must be at the given location at the given time instance.
         * 
         * @param _key the key representing the location of the actor.
         * @param _t the time instance the actor must be at the location.
         */
        void insert_landmark(int _index_point, double _t);


        /**
         * 
         */
        // inline size_t get_index_edge

    private:
        // size_t size_map_threshold = 10000; // threshold for switching between small and large cat

        std::unordered_map<int, std::list<std::pair<double, double>>> cat;
        // std::vector<std::list<std::pair<std::shared_ptr<Key3D>, std::pair<double, double>>>> cat_large; // 
        // std::vector<std::list<std::pair<double, double>>> cat_small; //

        std::shared_ptr<Instance> instance; // shared pointer to graph instance
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_CONSTRAINT_TABLE_HPP
