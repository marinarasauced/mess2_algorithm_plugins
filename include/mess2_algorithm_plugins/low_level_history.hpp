#ifndef MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HISTORY_HPP
#define MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HISTORY_HPP

#include "mess2_algorithm_plugins/common.hpp"

namespace mess2_algorithms
{
    /**
     * @brief defines a low level history element constiting of the cummulative score and time, the index of the parent vertex, the index of the previous parent history, and the edge transition type.
     */
    struct low_level_history_element {
        double score;
        double time;
        int64_t index_parent;
        int64_t index_history;
        std::string type;
    };

    /**
     * @brief alias for the structure of the low level history.
     */
    using low_level_history = std::vector<low_level_history_element>;

    /**
     * @class LowLevelHistory
     * @brief manages the history for the low level search algorithm.
     * 
     * this class provides the tools to manage a low level history for branches with repeat exploration of vertices and edges.
     */
    class LowLevelHistory
    {
    public:
        /**
         * @brief constructs a class instance of LowLevelHistory.
         */
        LowLevelHistory();

        /**
         * @brief add a history element to the history.
         * 
         * this method appends q history element to the currently existing history.
         * 
         * @param score the cummulative score of the current branch.
         * @param time the cummulative time of the current branch.
         * @param index_parent the index of the parent vertex.
         * @param index_history the index of the previous parent history.
         * @param type the type of transition; i.e., wait, rotate, translate.
         */
        void append_history(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history, const std::string& type);

        /**
         * @brief clears the history.
         * 
         * this method clears the history.
         */
        void clear_history();

        /**
         * @brief looks up the associated history of an index.
         * 
         * this method looks up the history element of an index from the currently explored branches.
         * 
         * @param index_history the index of the history element that is to be looked up.
         * @return a low level history element.
         */
        low_level_history_element lookup_history(const int64_t& index_history);

        /**
         * @brief gets the size of the history.
         * 
         * this methods returns the current size of the history.
         */
        int64_t size_history();

        /**
         * 
         */
        double unpack_score(const int64_t& index_history);

        /**
         * 
         */
        std::vector<std::pair<double, int64_t>> unpack_path(const int64_t& index_history);

        /**
         * @brief prints information about the current class.
         */
        void info();

    private:
        low_level_history history_;
    };
}

#endif // MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HISTORY_HPP