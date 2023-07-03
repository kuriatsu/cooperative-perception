#include <despot/planner.h>
#include "ras.h"
#include "ras_world.h"

using namespace despot;

class MyPlanner: public Planner {
public:
	MyPlanner() {
	}

	DSPOMDP* InitializeModel(option::Option* options) {
		DSPOMDP* model = new Ras();
		return model;
	}

	World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options) {
        SumoWorld* world = new SumoWorld();
        world->connect();
        world->Initialize();
        world_type = "simulator";
        return world;
	}

    void InitializeDefaultParameters() {
	}

	std::string ChooseSolver() {
		return "DESPOT";
	}

    void PlanningLoop(Solver*& solver, World* world, Logger* logger) {
        for (int i=0; i < Global::config.sim_len; i++) {
            bool terminal = RunStep(solver, world, logger);
            if (terminal) break;
        }
    }

    bool RunStep(Solver* solver, World* world, Logger* logger) {
        logger->CheckTargetTime();

        double step_start_t = get_time_second();
        double start_t = get_time_second();
        ACT_TYPE action = solver->Search().action;
        double end_t = get_time_second();
        double search_time = end_t - start_t;

        OBS_TYPE obs;
        double start_t = get_time_second();
        bool terminal = world->ExecuteAction(action, obs);
        double end_t = get_time_second();
        double execute_time = end_t - start_t;

        double start_t = get_time_second();
        solver->BeliefUpdate(action, obs);
        double end_t = get_time_second();
        double update_time = end_t - start_t;

};

int main(int argc, char* argv[]) {
	return MyPlanner().RunPlanning(argc, argv);
}
