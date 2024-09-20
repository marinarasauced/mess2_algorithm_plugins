#ifndef MESS2_ALGORITHM_PLUGINS_HEURISTIC_HPP
#define MESS2_ALGORITHM_PLUGINS_HEURISTIC_HPP

#include "mess2_algorithm_plugins/common.hpp"

#include "mess2_algorithm_plugins/graph/_graph.hpp"

namespace mess2_algorithms
{
    class Heuristic
    {
    public:
        Heuristic();

        void heuristic_fill(const std::vector<double>& scores, const Graph& graph, const int64_t index_target);
        double heuristic_lookup(const int64_t index_heuristic);

    private:
        std::vector<double> distances_;     // octile distances
    };

    Heuristic generate_heuristic(const Graph& graph, const std::string& type);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_HEURISTIC_HPP
