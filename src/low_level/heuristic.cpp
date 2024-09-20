
#include "mess2_algorithm_plugins/low_level/heuristic.hpp"

namespace mess2_algorithms
{
    Heuristic::Heuristic() {};

    void Heuristic::heuristic_fill(const std::vector<double>& scores, const Graph& graph, const int64_t index_target)
    {
        const double scale = *std::min_element(scores.begin(), scores.end());
        const double d1_ = 1.0;
        const double d2_ = std::sqrt(2.0);

        const auto edges = graph.get_edges();
        const auto vertices = graph.get_vertices();
        // admissible_ = 
        const auto vertex_target = vertices[index_target];
        const auto x_target = vertex_target.get_x();
        const auto y_target = vertex_target.get_y();

        const auto n_edges = static_cast<int64_t>(edges.size());
        distances_.resize(n_edges);

        // http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
        for (int64_t iter = 0; iter < n_edges; ++iter) {
            const auto edge = edges[iter];
            const auto index_source = edge.get_index_child();
            const auto vertex_source = vertices[index_source];
            const auto x_source = vertex_source.get_x();
            const auto y_source = vertex_source.get_y();

            const auto dx = std::abs(x_source - x_target);
            const auto dy = std::abs(y_source - y_target);

            const auto distance = scale * (d1_ * (dx + dy) + (d2_ - 2 * d1_) * std::min(dx, dy));
            distances_[iter] = distance;
        }
    }

    double Heuristic::heuristic_lookup(const int64_t index_heuristic) {
        return distances_[index_heuristic];
    }

} // namespace mess2_algorithms
