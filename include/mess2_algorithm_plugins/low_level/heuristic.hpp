#ifndef MESS2_ALGORITHM_PLUGINS_HEURISTIC_HPP
#define MESS2_ALGORITHM_PLUGINS_HEURISTIC_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/graph.hpp"

namespace mess2_algorithms
{
    class Heuristic
    {
    public:
        Heuristic();

        void fill_heuristic(const std::vector<double>& scores, const Graph& graph, const int64_t index_target);
        double lookup_heuristic(const int64_t index_heuristic);

    private:
        std::vector<double> distances_;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_HEURISTIC_HPP
