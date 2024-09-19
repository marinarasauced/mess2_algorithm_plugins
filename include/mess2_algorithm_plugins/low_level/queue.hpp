#ifndef MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_QUEUE_HPP
#define MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_QUEUE_HPP

#include "mess2_algorithm_plugins/common.hpp"

namespace mess2_algorithms
{
    struct low_level_queue {
        double score;
        double time;
        int64_t index_parent;
        int64_t index_history;
    };

    struct low_level_queue_operator {
        bool operator()(const low_level_queue& a, const low_level_queue& b) {
            if (a.score != b.score) {
                return a.score < b.score;
            }
            return a.time > b.time;
        }
    };

    class LowLevelQueue
    {
    public:
        LowLevelQueue();

        void queue_append(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history);
        void queue_clear();
        low_level_queue queue_lookup();   

        bool is_empty(); 
    
    private:
        void queue_pop();
    
        std::priority_queue<low_level_queue, std::vector<low_level_queue>, low_level_queue_operator> queue_;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_QUEUE_HPP