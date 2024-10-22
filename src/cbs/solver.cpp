
#include "mess2_algorithm_plugins/cbs/solver.hpp"

namespace mess2_algorithms
{
    CBS::CBS(std::shared_ptr<Instance> &_instance, bool _sipp, int _screen) : screen(_screen), focal_w(1.0), instance(_instance)
    // mdd_helper(initial_constraints, search_engines),
    // rectangle_helper(instance),
    // mutex_helper(instance, initial_constraints),
    // corridor_helper(search_engines, initial_constraints),
    // heuristic_helper(instance.getDefaultNumberOfAgents(), paths, search_engines, initial_constraints, mdd_helper)
    {
        clock_t t = std::clock();
        initial_constraints.resize(instance->n_actors, ConstraintTable());
        search_engines.resize(instance->n_actors);
        for (int iter = 0; iter < instance->n_actors; ++iter) {
            if (_sipp) {
                // search_engines[iter] == std::make_shared<SIPP>();
                throw std::runtime_error("sipp not implemented yet");
            } else {
                search_engines[iter] = std::make_shared<SpaceTimeAStar>(instance, iter);
            }
            runtime_preprocessing = (double) (std::clock() - t) / CLOCKS_PER_SEC;

            // mutex_helper.search_engines = search_engines;

            // screen for debugging omitted
        }
    };

    void CBS::generate_root()
    {
        dummy_start = std::make_shared<CBSNode>();
        dummy_start->g_cummulative = 0.0;
        paths.resize(instance->n_actors, nullptr);

        // mdd_helper.init(num_of_agents);
    	// heuristic_helper.init();

        if (paths_found_initially.empty())
        {
            paths_found_initially.resize(instance->n_actors);

            // generate a random permutation of actor instances
            std::vector<int> actors(instance->n_actors);
            for (int iter = 0; iter < instance->n_actors; ++iter) {
                actors[iter] = iter;
            }

            if (random_root) {
                std::random_device rd;
                std::mt19937 g(rd());
                std::shuffle(std::begin(actors), std::end(actors), g);
            }

            for (auto i : actors) {
                paths_found_initially[i] = search_engines[i]->find_path(dummy_start, initial_constraints[i], paths, 0.0);

                std::cout << "n_path : " << paths_found_initially[i].size() << std::endl;
                std::cout << "n_generated : " << search_engines[i]->n_generated << std::endl;
                std::cout << "n_expanded : " << search_engines[i]->n_expanded << std::endl;
            }
        }

    }

    bool CBS::solve(double _time_limit, double _cost_lowerbound, double _cost_upperbound)
    {
        this->score_min = _cost_lowerbound;
        this->cost_upperbound = _cost_upperbound;
        this->time_limit = _time_limit;

        // screen for debugging omitted

        time_start = std::clock();

        (void) generate_root();





        return false;
    }

} // namespace mess2_algorithms
