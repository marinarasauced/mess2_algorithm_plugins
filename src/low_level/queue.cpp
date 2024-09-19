
#include "mess2_algorithm_plugins/low_level/queue.hpp"

namespace mess2_algorithms
{
    LowLevelQueue::LowLevelQueue() {};

    void LowLevelQueue::queue_append(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history) {
        queue_.emplace(low_level_queue{score, time, index_parent, index_history});
    }

    void LowLevelQueue::queue_clear() {
        std::priority_queue<low_level_queue, std::vector<low_level_queue>, low_level_queue_operator> queue_empty;
        std::swap(queue_, queue_empty);
    }

    low_level_queue LowLevelQueue::queue_lookup() {
        if (queue_.empty()) {
            throw std::runtime_error("attempt to get from an empty queue.");
        }
        return queue_.top();
    }

    void LowLevelQueue::queue_pop() {
        queue_.pop();
    }

} // namespace mess2_algorithms
