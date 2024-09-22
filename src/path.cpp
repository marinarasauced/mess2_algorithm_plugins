
#include "mess2_algorithm_plugins/path.hpp"

namespace mess2_algorithms
{
    pathplan retrieve_path(LowLevelHistory& history)
    {
        const auto size = history.size_history();
        if (size <= 0) {
            throw std::runtime_error("history cannot be empty");
        }

        const auto last = history.lookup_history(size - 1);
        pathplan path;
        path.first = last.score;
        path.second.emplace_back(std::pair<double, int64_t>({last.time, last.index_parent}));

        // path.second.emplace({last.time, last.index_parent});
        auto index_history_curr = last.index_history;
        while (index_history_curr != -1) {
            auto curr = history.lookup_history(index_history_curr);
            path.second.emplace_back(std::pair<double, int64_t>({curr.time, curr.index_parent}));
            
            index_history_curr = curr.index_history;
        }

        std::reverse(path.second.begin(), path.second.end());

        return path;
    }

    void print_path(const pathplan& path, const std::vector<Vertex>& vertices)
    {
        const auto score = path.first;
        const auto points = path.second;

        std::cout << "final score : " << score <<std::endl;
        std::cout << "final path : " << std::endl;
        for (const auto& point : points) {
            const auto time = point.first;
            const auto index = point.second;
            const auto vertex = vertices[index];
            
            std::cout << "\t" << time << ", " << index << " (" << vertex.get_x() << ", " << vertex.get_y() << ", " << vertex.get_theta() << ")" << std::endl;
        }
    }

} // namespace mess2_algorithms
