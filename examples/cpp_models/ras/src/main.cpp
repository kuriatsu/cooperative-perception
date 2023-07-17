#include <despot/planner.h>
#include "task_allocation.h"
#include "ras_world.h"
#include "operator_model.h"
#include "sumo_interface.h"

using namespace despot;

class MyPlanner: public Planner {
public:
	MyPlanner() {
	}

    // params
    int planning_horizon = 150;
    double risk_thresh = 0.5;
    // operator_model
    double min_time = 3.0;
    double acc_time_min = 0.5;
    double acc_time_slope = 0.25;

    // vehicle model
    double max_speed = 11.2;
    double yield_speed = 2.8;
    double max_accel = 0.15 * 9.8;
    double max_decel = 0.2 * 9.8;
    int safety_margin = 5;
    double delta_t = Globals::config.time_per_move;

    // sim model
    double obstacle_density = 0.1; // 1ppl per 1m
    std::vector<double> perception_range = {50, 150}; // left+right range, forward range

    // model parameters
    string world_type = "simulator";
    string belief_type = "DEFAULT";

    // models
    OperatorModel *operator_model = new OperatorModel(min_time, acc_time_min, acc_time_slope);
    VehicleModel *vehicle_model = new VehicleModel(max_speed, yield_speed, max_accel, max_decel, safety_margin, delta_t);

	DSPOMDP* InitializeModel(option::Option* options) {
		DSPOMDP* model = new TaskAllocation(planning_horizon, max_speed, yield_speed, risk_thresh, vehicle_model, operator_model);
		return model;
	}

	World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options) {
        RasWorld* ras_world = new RasWorld(max_speed, yield_speed, max_accel, max_decel, safety_margin, delta_t, obstacle_density, perception_range);
        ras_world->operator_model = operator_model;
        ras_world->Connect();
        ras_world->Initialize();
        return ras_world;
	}

    void InitializeDefaultParameters() {
        Globals::config.sim_len = 1;
	}

	std::string ChooseSolver() {
		return "DESPOT";
	}

    void PlanningLoop(Solver*& solver, World* world, DSPOMDP* model, Logger* logger) {
        for (int i=0; i < Globals::config.sim_len; i++) {
            bool terminal = RunStep(solver, world, model, logger);
            if (terminal) break;
        }
    }

    bool RunStep(Solver* solver, World* world, DSPOMDP* model, Logger* logger) {
        RasWorld* ras_world = static_cast<RasWorld*>(world);
        TaskAllocation* ta_model = static_cast<TaskAllocation*>(model);
        logger->CheckTargetTime();

        ras_world->Step();

        TAState* start_state = static_cast<TAState*>(ras_world->GetCurrentState());
        ta_model->syncCurrentState(start_state);

        Belief* belief = ta_model->InitialBelief(start_state, belief_type);
        assert(belief != NULL);
        solver->belief(belief);

        double step_start_t = get_time_second();
        double start_t = get_time_second();
        ACT_TYPE action = solver->Search().action;
        double end_t = get_time_second();
        double search_time = end_t - start_t;

        OBS_TYPE obs;
        start_t = get_time_second();
        bool terminal = ras_world->ExecuteAction(action, obs);
        end_t = get_time_second();
        double execute_time = end_t - start_t;

        start_t = get_time_second();
        solver->BeliefUpdate(action, obs);
        end_t = get_time_second();
        double update_time = end_t - start_t;

        ras_world->UpdateState(action, obs, ta_model->getRiskProb(belief));

        return true;
    }

    
    int RunPlanning(int argc, char *argv[]) {

        /* =========================
         * initialize parameters
         * =========================*/
        string solver_type = ChooseSolver(); //"DESPOT";
        bool search_solver;
        int num_runs = 1;
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

        Belief *belief = NULL;
        Solver *solver = InitializeSolver(model, belief, solver_type, options);

        Logger *logger = NULL;
        InitializeLogger(logger, options, model, belief, solver, num_runs,
                main_clock_start, world, world_type, time_limit, solver_type);

        DisplayParameters(options, model);

        logger->InitRound(world->GetCurrentState());
        round_=0; step_=0;
        PlanningLoop(solver, world, model, logger);
        logger->EndRound();

        PrintResult(1, logger, main_clock_start);

        return 0;
    }
};

int main(int argc, char* argv[]) {
	return MyPlanner().RunPlanning(argc, argv);
}
