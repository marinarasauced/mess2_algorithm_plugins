
#include "mess2_algorithm_plugins/low_level_history.hpp"

namespace mess2_algorithms
{
    LowLevelHistory::LowLevelHistory() {};

    void LowLevelHistory::append_history(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history, const std::string& type)
    {
        const low_level_history_element element = {score, time, index_parent, index_history, type};
        history_.push_back(element);
    }

    void LowLevelHistory::clear_history()
    {
        history_.clear();
    }

    low_level_history_element LowLevelHistory::lookup_history(const int64_t& index_history)
    {
        if (index_history > size_history()) {
            throw std::out_of_range("history index cannot exceed size of history");
        }

        const low_level_history_element element = history_[index_history];
        return element;
    }

    int64_t LowLevelHistory::size_history()
    {
        return static_cast<int64_t>(history_.size());
    }

    double LowLevelHistory::unpack_score(const int64_t& index_history)
    {
        if (index_history > size_history()) {
            throw std::out_of_range("history index cannot exceed size of history");
        }

        return history_[index_history].score;
    }

    std::vector<std::pair<double, int64_t>> LowLevelHistory::unpack_path(const int64_t& index_history)
    {
        if (index_history > size_history()) {
            throw std::out_of_range("history index cannot exceed size of history");
        }

        std::vector<std::pair<double, int64_t>> path;
        const auto last = lookup_history(index_history);
        path.emplace_back(std::pair<double, double>({last.time, last.index_parent}));

        auto index_curr = last.index_history;
        while (index_curr != -1) {
            auto curr = lookup_history(index_curr);
            path.emplace_back(std::pair<double, double>({curr.time, curr.index_parent}));
            index_curr = curr.index_history;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    void LowLevelHistory::info()
    {
        throw std::runtime_error("low level history info not implemented");
    }

} // namespace mess2_algorithms
