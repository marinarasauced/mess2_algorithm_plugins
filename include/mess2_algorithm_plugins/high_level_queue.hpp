#ifndef MESS2_ALGORITHM_PLUGINS_HIGH_LEVEL_QUEUE_HPP
#define MESS2_ALGORITHM_PLUGINS_HIGH_LEVEL_QUEUE_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/constraint.hpp"

namespace mess2_algorithms
{
    /**
     * @brief defines a high level queue element consisting of the cummulative score of all actors from the parent and constraints for all actors.
     */
    struct high_level_queue_element {
        int64_t n_constraints;
        double score;
        std::vector<ConstraintsVertices> constraints;
    };

    /**
     * @brief defines the custom operator used to sort queue elements such that those with the lowest cummulative score are explored first.
     * 
     * this operators sorts by cummulative score first and is first come first server second.
     */
    struct high_level_queue_operator {
        bool operator() (const high_level_queue_element& a, const high_level_queue_element& b) {
            if (a.n_constraints != b.n_constraints) {
                return a.n_constraints > b.n_constraints;
            }
            return a.score > b.score;
        }
    };

    /**
     * @brief alias for the structure of the high level priority queue.
     */
    using high_level_queue = std::priority_queue<high_level_queue_element, std::vector<high_level_queue_element>, high_level_queue_operator>;

    /**
     * @class HighLevelQueue
     * @brief manages the queue for the high level search algorithm.
     * 
     * this class provides the tools to manage a high level priority queue consisting of a constraint tree for actors in a given scenario.
     */
    class HighLevelQueue
    {
    public:
        /**
         * @brief constructs a class instance of HighLevelQueue.
         */
        HighLevelQueue();

        /**
         * @brief adds a queue element to the queue.
         * 
         * this method appends a queue element to the priority queue for future iterations.
         * 
         * @param score the cummulative score of scores of actors in the parent path plans.
         * @param constraints the tree of constraints for actors for future iterations.
         */
        void append_queue(const int64_t& n_constraints, const double& score, const std::vector<ConstraintsVertices>& constraints);

        /**
         * @brief clears the queue.
         * 
         * this method clears the queue by replacing it with an empty equivalent.
         */
        void clear_queue();

        /**
         * @brief looks up the next queue element to explore.
         * 
         * this method looks up the next queue element for the high level algorithm to explore and then removes that element from the queue.
         * 
         * @return a high level queue element
         */
        high_level_queue_element lookup_queue();

        /**
         * @brief gets the size of the queue.
         * 
         * this methods returns the current size of the queue.
         */
        int64_t size_queue();

        /**
         * @brief prints informaton about the current class.
         */
        void info();

    private:
        high_level_queue queue_;
    };
}

#endif // MESS2_ALGORITHM_PLUGINS_HIGH_LEVEL_QUEUE_HPP