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
        SumoWorld* world = new RasWorld();
        world->connect();
        world->Initialize();
        world_type = "simulator";
        return world;
	}

    void InitializeDefaultParameters() {
        Globals::config.sim_len = 1;
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

        sim.step();
        auto targets = sim.perception();
        sim.controlEgoVehicle(targets);

        Belief* belief = model->InitialBelief(world->GetCurrentState(), belief_type);
        assert(belief != NULL);
        solver->belief(belief);

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
    }

    
    int Planner::RunPlanning(int argc, char *argv[]) {

        /* =========================
         * initialize parameters
         * =========================*/
        string solver_type = ChooseSolver(); //"DESPOT";
        bool search_solver;
        int num_runs = 1;
        string world_type = "pomdp";
        string belief_type = "DEFAULT";
        int time_limit = -1;

        option::Option *options = InitializeParamers(argc, argv, solver_type,
                search_solver, num_runs, world_type, belief_type, time_limit);
        if(options==NULL)
            return 0;
        clock_t main_clock_start = clock();


        DSPOMDP *model = InitializeModel(options);
        assert(model != NULL);


        World *world = InitializeWorld(world_type, model, options);
        assert(world != NULL);

        Belief belief = NULL;
        Solver *solver = InitializeSolver(model, belief, solver_type, options);

        Logger *logger = NULL;
        InitializeLogger(logger, options, model, belief, solver, num_runs,
                main_clock_start, world, world_type, time_limit, solver_type);

        DisplayParameters(options, model);

        logger->InitRound(world->GetCurrentState());
        round_=0; step_=0;
        PlanningLoop(solver, world, logger);
        logger->EndRound();

        PrintResult(1, logger, main_clock_start);

        return 0;
    }
};

int main(int argc, char* argv[]) {
	return MyPlanner().RunPlanning(argc, argv);
}
