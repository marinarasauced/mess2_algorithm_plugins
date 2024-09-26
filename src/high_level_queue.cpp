
#include "mess2_algorithm_plugins/high_level_queue.hpp"

namespace mess2_algorithms
{
    HighLevelQueue::HighLevelQueue() {};

    void HighLevelQueue::append_queue(const double& score, const std::vector<ConstraintsVertices>& constraints)
    {
        const high_level_queue_element element = {score, constraints};
        queue_.push(element);
    }

    void HighLevelQueue::clear_queue()
    {
        high_level_queue empty;
        std::swap(queue_, empty);
    }

    high_level_queue_element HighLevelQueue::lookup_queue()
    {
        if (size_queue() == 0) {
            throw std::runtime_error("queue cannot be empty during lookup");
        }
        const high_level_queue_element element = queue_.top();
        (void) queue_.pop();
        return element;
    }

    int64_t HighLevelQueue::size_queue()
    {
        return static_cast<int64_t>(queue_.size());
    }

    void HighLevelQueue::info()
    {
        throw std::runtime_error("high level queue info not implemented");
    }

} // namespace mess2_algorithms
