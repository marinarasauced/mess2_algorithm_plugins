
#include "mess2_algorithm_plugins/low_level/history.hpp"

namespace mess2_algorithms
{
    LowLevelHistory::LowLevelHistory() {};

    void LowLevelHistory::history_append(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history, const std::string& type) {
        history_.emplace_back(low_level_history{score, time, index_parent, index_history, type});
    }

    void LowLevelHistory::history_clear() {
        history_.clear();
    }

    low_level_history LowLevelHistory::history_lookup(const int64_t& index_history) {
        if (index_history < 0 || index_history >= static_cast<int64_t>(history_.size())) {
            throw std::out_of_range("history index out of range in get_history.");
        }
        return history_[index_history];
    }

    int64_t LowLevelHistory::history_size() {
        return static_cast<int64_t>(history_.size());
    }

} // namespace mess2_algorithms
