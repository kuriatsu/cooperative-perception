#include <despot/planner.h>

#include "task_allocation.h"

using namespace despot;

class MyPlanner: public Planner {
public:
    MyPlanner() {
    }


    DSPOMDP* InitializeModel(option::Option* options) {
		DSPOMDP* model = new TaskAllocation();
        return model;
    }

    World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options)
    {
        return InitializePOMDPWorld(world_type, model, options);
    }

    void InitializeDefaultParameters() {
        Globals::config.num_scenarios = 500;
        Globals::config.sim_len = 150;
        Globals::config.search_depth = 50;
        Globals::config.time_per_move = 1;
    }

    std::string ChooseSolver(){
        return "DESPOT";
    }
    };

    int main(int argc, char* argv[]) {
        return MyPlanner().RunEvaluation(argc, argv);
    }
