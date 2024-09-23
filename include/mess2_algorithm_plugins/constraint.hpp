#ifndef MESS2_ALGORITHM_PLUGINS_CONSTRAINT_HPP
#define MESS2_ALGORITHM_PLUGINS_CONSTRAINT_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/graph.hpp"
// #include "mess2_algorithm_plugins/low_level/history.hpp"

namespace mess2_algorithms
{
    struct constrained {
        double t_init;
        double t_term;
    };

    class Constraints
    {
    public:
        Constraints();

        void fill_constraints(const Graph& graph);

        void add_constraint_to_vertex(const int64_t& index_vertex, const double& t_init, const double& t_term);
        std::vector<constrained> lookup_constraints_at_vertex(const int64_t& index_vertex) const;

    private:
        std::vector<std::vector<constrained>> constraints_;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_CONSTRAINT_HPP
