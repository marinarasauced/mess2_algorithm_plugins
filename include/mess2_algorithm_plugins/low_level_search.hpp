#ifndef MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_SEARCH_HPP
#define MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_SEARCH_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/actor.hpp"
#include "mess2_algorithm_plugins/constraint.hpp"
#include "mess2_algorithm_plugins/graph.hpp"
#include "mess2_algorithm_plugins/low_level_history.hpp"
#include "mess2_algorithm_plugins/low_level_queue.hpp"

namespace mess2_algorithms
{
    /**
     * @brief defines the output of a low level search.
     */
    struct low_level_search_output {
        double score;
        std::vector<std::pair<double, int64_t>> path;
    };

    /**
     * @class LowLevelSearch
     * @brief manages the low level search using an a* variation for an actor in a grid of vertices and edges.
     * 
     * this class provides the tools to execute a low level search for a single acotr in a threat environment, including executing the search algorithm and displaying info about the search.
     */
    class LowLevelSearch
    {
    public:
        /**
         * @brief constructs a class instance of LowLevelSearch.
         */
        LowLevelSearch();

        /**
         * @brief executes a low level search.
         * 
         * this method executes a low level search for a single actor.
         * 
         * @param graph the graph about which the search is to be executed on.
         * @param actor the actor about which the search is to be executed for.
         * @param constraints the constraints of the actor for vertices in the graph.
         * @param timeout the seconds until the function timeout occurs (default = 5.0).
         * @return a low level search output.
         */
        low_level_search_output execute_low_level_search(const Graph& graph, const Actor& actor, const ConstraintsVertices& ct, const int64_t& n_iters = 10000000);

        /**
         * 
         */
        void info();

    private:
        LowLevelHistory history_;
        LowLevelQueue queue_;
        std::vector<int64_t> map_;
    };

    /**
     * 
     */
    void info_low_level_search_output(const low_level_search_output& output, const Graph& graph, Actor& actor);

    /**
     * 
     */
    void save_low_level_search_output(const std::string& path_goal, const low_level_search_output& path, const Graph& graph, const bool& use_simplify);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_SEARCH_HPP