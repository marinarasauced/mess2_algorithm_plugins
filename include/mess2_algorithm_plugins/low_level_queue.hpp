#ifndef MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_QUEUE_HPP
#define MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_QUEUE_HPP

#include "mess2_algorithm_plugins/common.hpp"

namespace mess2_algorithms
{
    /**
     * @brief defines a low level queue element consisting of the cummulative score and time, the index of the parent vertex, and the history index.
     */
    struct low_level_queue_element {
        double score;
        double time;
        int64_t index_parent;
        int64_t index_history;
        int64_t n_visits;
    };

    /**
     * @brief defines a custom operator used to sort queue elements such that those with the lowest cummulative score are explored first.
     * 
     * this operator sorts by cummulative score first and cummulative time second.
     */
    struct low_level_queue_operator {
        bool operator() (const low_level_queue_element& a, const low_level_queue_element& b) {
            if (a.score != b.score) {
                return a.score > b.score;
            }
            return a.time > b.time;
        }
    };

    /**
     * @brief alias for the structure of the low level priority queue.
     */
    using low_level_queue = std::priority_queue<low_level_queue_element, std::vector<low_level_queue_element>, low_level_queue_operator>;

    /**
     * @class LowLevelQueue
     * @brief manages the queue for the low level search algorithm.
     * 
     * this class provides the tools to manage a low level priority queue for a graph of vertices, edges, and adjacencies.
     */
    class LowLevelQueue
    {
    public:
        /**
         * @brief constructs a class instance of LowLevelQueue.
         */
        LowLevelQueue();

        /**
         * @brief adds a queue element to the queue.
         * 
         * this method appends a queue element to the priority queue for future iterations.
         * 
         * @param score the cummulative score of the current branch.
         * @param time the cummulative time of the current branch.
         * @param index_parent the index of the parent vertex.
         * @param index_history the index of the parent history.
         */
        void append_queue(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history, const int64_t& n_visits);

        /**
         * @brief clears the queue.
         * 
         * this method clears the queue by replacing it with an empty equivalent.
         */
        void clear_queue();

        /**
         * @brief looks up the next queue element to explore.
         * 
         * this method looks up the next queue element for the low level algorithm to explore and then removes that element from the queue.
         * 
         * @return a low level queue element
         */
        low_level_queue_element lookup_queue();

        /**
         * @brief gets the size of the queue.
         * 
         * this methods returns the current size of the queue.
         */
        int64_t size_queue();

        /**
         * @brief prints information abour the current class.
         */
        void info();

    private:
        low_level_queue queue_;
    };
}

#endif // MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_QUEUE_HPP