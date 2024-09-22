
#include "mess2_algorithm_plugins/low_level/heuristic.hpp"

namespace mess2_algorithms
{
    Heuristic::Heuristic() {};

    void Heuristic::fill_heuristic(const std::vector<double>& scores, const Graph& graph, const int64_t index_target)
    {
        const auto edges = graph.get_edges();
        const auto vertices = graph.get_vertices();
        const auto vertex_target = vertices[index_target];
        const auto x_target = vertex_target.get_x();
        const auto y_target = vertex_target.get_y();

        const auto n_edges = static_cast<int64_t>(edges.size());
        distances_.resize(n_edges);

        std::vector<double> distances(n_edges);

        for (int64_t iter = 0; iter < n_edges; ++iter) {
            const auto edge = edges[iter];
            const auto index_source = edge.get_index_child();
            const auto vertex_source = vertices[index_source];
            const auto x_source = vertex_source.get_x();
            const auto y_source = vertex_source.get_y();

            const auto distance = std::sqrt(
                std::pow(x_source - x_target, 2) +
                std::pow(y_source - y_target, 2)
            );
            distances[iter] = distance;
        }

        const auto scale = *std::min_element(distances.begin(), distances.end());

        for (int64_t iter = 0; iter < n_edges; ++iter) {
            distances_[iter] = distances[iter] * (scores[iter] / scale);
        }
    }

    double Heuristic::lookup_heuristic(const int64_t index_heuristic) {
        return distances_[index_heuristic];
    }

} // namespace mess2_algorithms
