#ifndef MESS2_ALGORITHM_PLUGINS_HIGH_LEVEL_SEARCH_HPP
#define MESS2_ALGORITHM_PLUGINS_HIGH_LEVEL_SEARCH_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/actor.hpp"
#include "mess2_algorithm_plugins/constraint.hpp"
#include "mess2_algorithm_plugins/high_level_queue.hpp"
#include "mess2_algorithm_plugins/low_level_history.hpp"
#include "mess2_algorithm_plugins/low_level_queue.hpp"
#include "mess2_algorithm_plugins/low_level_search.hpp"

namespace mess2_algorithms
{
    /**
     * @brief defines the output of a high level search.
     */
    struct high_level_search_output {

    };

    /**
     * @class HighLevelSearch
     * @brief manages the high level search of a conflict based search for multi actor path planning.
     * 
     * this class provides the tools to execute a high level cbs search for multiple actors in a threat environment, including executing the algorithm and displaying info about the search.
     */
    class HighLevelSearch
    {
    public:
        /**
         * @brief constructs a class instance of HighLevelSearch.
         */
        HighLevelSearch();

        /**
         * @brief executes a high level search.
         * 
         * this method executes a high level search for multiple actors.
         * 
         * @param graph the graph upon which the actors path's are planned.
         * @param actors a list of actors containing pregenerated information for their respective searches on the graph.
         * @param timeout the seconds until the function timeout occurs (default : 20.0).
         * @return a high level search output.
         */
        high_level_search_output execute_high_level_search(const Graph& graph, std::vector<Actor>& actors, const double& timeout = 20.0);

        /**
         * 
         */
        bool execute_conflict_search(const double& score, const std::vector<std::vector<std::pair<double, int64_t>>>& paths, const Graph& graph, std::vector<Actor>& actors, std::vector<ConstraintsVertices>& constraints);

        /**
         * 
         */
        void execute_conflict_resolution(const double& score, const std::vector<ConstraintsVertices>& constraints);

        /**
         * @brief prints informaton about the current class.
         */
        void info();

    private:
        HighLevelQueue queue_;
        LowLevelSearch search_;
    };
}

#endif // MESS2_ALGORITHM_PLUGINS_HIGH_LEVEL_SEARCH_HPP