
#include "mess2_algorithm_plugins/constraint.hpp"

namespace mess2_algorithms
{
    Constraints::Constraints() {};

    void Constraints::fill_constraints(const Graph& graph) {
        constraints_.resize(graph.get_vertices().size());
    }

    void Constraints::add_constraint_to_vertex(const int64_t& index_vertex, const double& t_init, const double& t_term) {
        const constrained constraint = {t_init, t_term};
        constraints_[index_vertex].push_back(constraint);
    }

    std::vector<constrained> Constraints::lookup_constraints_at_vertex(const int64_t& index_vertex) const {
        return constraints_[index_vertex];
    }

} // namespace mess2_algorithms
