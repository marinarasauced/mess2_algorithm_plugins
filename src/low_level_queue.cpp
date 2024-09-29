
#include "mess2_algorithm_plugins/low_level_queue.hpp"

namespace mess2_algorithms
{
    LowLevelQueue::LowLevelQueue() {};

    void LowLevelQueue::append_queue(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history, const int64_t& n_visits)
    {
        const low_level_queue_element element = {score, time, index_parent, index_history, n_visits};
        queue_.push(element);
    }

    void LowLevelQueue::clear_queue()
    {
        low_level_queue empty;
        std::swap(queue_, empty);
    }

    low_level_queue_element LowLevelQueue::lookup_queue()
    {
        if (size_queue() == 0) {
            throw std::runtime_error("queue cannot be empty during lookup");
        }
        const low_level_queue_element element = queue_.top();
        (void) queue_.pop();
        return element;
    }

    int64_t LowLevelQueue::size_queue()
    {
        return static_cast<int64_t>(queue_.size());
    }

    void LowLevelQueue::info()
    {
        throw std::runtime_error("low level queue info not implemented");
    }

} // namespace mess2_algorithms