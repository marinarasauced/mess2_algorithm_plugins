#ifndef MESS2_ALGORITHM_PLUGINS_PATH_HPP
#define MESS2_ALGORITHM_PLUGINS_PATH_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/vertex.hpp"
#include "mess2_algorithm_plugins/low_level/history.hpp"

namespace mess2_algorithms
{
    using pathplan = std::pair<double, std::vector<std::pair<double, int64_t>>>;

    std::pair<double, std::vector<std::pair<double, int64_t>>> retrieve_path(LowLevelHistory& history);

    void print_path(const pathplan& path, const std::vector<Vertex>& vertices);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_PATH_HPP
