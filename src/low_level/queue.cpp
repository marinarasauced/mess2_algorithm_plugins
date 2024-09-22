
#include "mess2_algorithm_plugins/low_level/queue.hpp"

namespace mess2_algorithms
{
    LowLevelQueue::LowLevelQueue() {};

    void LowLevelQueue::append_queue(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history) {
        queue_.emplace(low_level_queue{score, time, index_parent, index_history});
    }

    void LowLevelQueue::clear_queue() {
        std::priority_queue<low_level_queue, std::vector<low_level_queue>, low_level_queue_operator> queue_empty;
        std::swap(queue_, queue_empty);
    }

    low_level_queue LowLevelQueue::lookup_queue() {
        if (queue_.empty()) {
            throw std::runtime_error("attempt to get from an empty queue");
        }
        const auto queue_top = queue_.top();
        (void) pop_queue();
        return queue_top;
    }

    bool LowLevelQueue::is_empty() {
        return queue_.empty();
    }

    void LowLevelQueue::pop_queue() {
        queue_.pop();
    }

} // namespace mess2_algorithms
