#include <despot/planner.h>
#include "task_allocation.h"
#include "ras_world.h"
#include "operator_model.h"
#include "sumo_interface.h"

using namespace despot;

class MyPlanner: public Planner {

private:
    // sim model
    double _obstacle_density = 0.01; // density 1ppl/m, 0.1=1ppl per 10m, 0.01=1ppl per 100m
    std::vector<double> _perception_range = {50, 150}; // left+right range, forward range
                                                      
    // model parameters
    string _world_type = "simulator";
    string _belief_type = "DEFAULT";
    string _policy_type = "DESPOT"; // DESPOT, MYOPIC, EGOISTIC
    option::Option *_options;

    // log
    string _log_file = "";
    
    // simulation params
    int _planning_horizon = 150;
    double _risk_thresh = 0.5;
    // operator_model
    double _min_time = 3.0;
    double _acc_time_min = 0.5;
    double _acc_time_slope = 0.25;

    // vehicle model
    double _max_speed = 11.2;
    double _yield_speed = 2.8;
    double _max_accel = 0.15 * 9.8;
    double _max_decel = 0.3 * 9.8;
    double _min_decel = 0.2 * 9.8;
    int _safety_margin = 20;

    double _delta_t = 1.0;
    double _time_per_move = 1.0;

public:
	MyPlanner() {
	}

    // models
    OperatorModel *_operator_model = new OperatorModel(_min_time, _acc_time_min, _acc_time_slope);
    VehicleModel *_vehicle_model = new VehicleModel(_max_speed, _yield_speed, _max_accel, _max_decel, _min_decel, _safety_margin, _delta_t);

	DSPOMDP* InitializeModel(option::Option* options) {
		DSPOMDP* model = new TaskAllocation(_planning_horizon, _risk_thresh, _vehicle_model, _operator_model, _delta_t);
		return model;
	}

	World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options) {
        RasWorld* ras_world = new RasWorld(_vehicle_model, _operator_model, _delta_t, _obstacle_density, _perception_range, _policy_type);
        ras_world->Connect();
        if (_log_file.empty()) 
            ras_world->Initialize();
        else
            ras_world->Initialize(_log_file);

        return ras_world;
	}

    void InitializeDefaultParameters() {
        Globals::config.num_scenarios = 100;
        Globals::config.sim_len = 90;
        Globals::config.time_per_move = _time_per_move; 
	}

	std::string ChooseSolver() {
		return "DESPOT";
	}

    void PlanningLoop(Solver*& solver, World* world, DSPOMDP* model, Logger* logger) {
        bool terminal = false;
        while (!terminal) {
            terminal = RunStep(solver, world, model, logger);
        }
//        for (int i=0; i < Globals::config.sim_len; i++) {
//            bool terminal = RunStep(solver, world, model, logger);
//            if (terminal) break;
//        }
    }

    bool RunStep(Solver* solver, World* world, DSPOMDP* model, Logger* logger) {
        RasWorld* ras_world = static_cast<RasWorld*>(world);
        TaskAllocation* ta_model = static_cast<TaskAllocation*>(model);
        logger->CheckTargetTime();
        
        for (int i=0; i<delta_t; i++) {
            ras_world->Step(0);
        }

        if (ras_world->isTerminate()) {
            return true;
        }

        TAState* start_state = static_cast<TAState*>(ras_world->GetCurrentState());
        std::vector<double> likelihood_list = ras_world->GetPerceptionLikelihood();
        ta_model->syncCurrentState(start_state, likelihood_list);

        Belief* belief = ta_model->InitialBelief(start_state, likelihood_list, belief_type);
        assert(belief != NULL);
        // solver->belief(belief);

        solver = InitializeSolver(model, belief, ChooseSolver(), _options);

        ACT_TYPE action;
        if (_policy_type == "MYOPIC")
            action = ras_world->MyopicAction();
        else if (_policy_type == "EGOISTIC")
            action = ras_world->EgoisticAction();
        else
            action = solver->Search().action;

        OBS_TYPE obs;
        bool terminal = ras_world->ExecuteAction(action, obs);
        solver->BeliefUpdate(action, obs);
        ras_world->UpdateState(action, obs, ta_model->getRiskProb(belief));
        ras_world->Log(action, obs);

        return false;
    }

    
    int RunPlanning(const double obstacle_density, const std::string policy_type, const std::string log_file) {

        /* =========================
         * initialize parameters
         * =========================*/
        bool search_solver;
        int num_runs = 1;
        int time_limit = -1;
        _obstacle_density = obstacle_density;
        _policy_type = policy_type;
        _log_file = log_file;

        std::string solver_type = ChooseSolver();
        _options = InitializeParamers(argc, argv, solver_type,
                    search_solver, num_runs, _world_type, _belief_type, time_limit);
        if(_options==NULL)
            return 0;
        clock_t main_clock_start = clock();

        DSPOMDP *model = InitializeModel(_options);
        assert(model != NULL);

        World *world = InitializeWorld(world_type, model, _options);
        assert(world != NULL);

        Belief *belief = NULL;
        Solver *solver = NULL;

        Logger *logger = NULL;
        InitializeLogger(logger, _options, model, belief, solver, num_runs,
                main_clock_start, world, world_type, time_limit, solver_type);

        DisplayParameters(_options, model);

        logger->InitRound(world->GetCurrentState());
        round_=0; step_=0;
        PlanningLoop(solver, world, model, logger);
        logger->EndRound();

        PrintResult(1, logger, main_clock_start);
        
        std::stringstream ss;
        ss << policy_type+std::to_string(obstacle_density)+"_"; 

        if (log_file.empty) {
            time_t now = std::time(nullptr);
            struct tm* local_now = std::localtime(&now);
            ss << local_now->tm_year + 1900
               << setw(2) << setfill('0') << local_now->tm_mon 
               << setw(2) << setfill('0') << local_now->tm_mday 
               << setw(2) << setfill('0') << local_now->tm_hour
               << setw(2) << setfill('0') << local_now->tm_min
               << setw(2) << setfill('0') << local_now->tm_sec
               << ".json";
        }
        else {
            std::stringstream filename_ss{log_file};
            std::string filename_s;
            std::vector<std::string> filename_splitted;
            while (getline(filename_ss, s, "_")) {
                filename_splitted.emplace_back(s);
            }
            ss << filename_splitted[-1];
        }

        world->SaveLog(ss.str());

        std::cout << "#### close simulator ####" << std::endl;
        world->Close();

        return 0;
    }
};

int main(int argc, char* argv[]) {
    char *optarg;
    int optind, opterr, optopt;
    double obstacle_density;
    std::string policy_type, log_file;
	MyPlanner planner = MyPlanner();

    while ((opt = getopt(argc, argv, "d:p:l:")) != -1) {
        switch (opt) {
            case "d":
                obstacle_density = std::stod(std::string(optarg));
                break;
            case "p":
                policy_type = std::string(optarg);
                break;
            case "l":
                log_file = std::string(optarg);
                break;
            default:
                print("No option \n Usage: [-d dencity] [-p policy] [-l log_file]");
                break;
        }
    }

    return planner.RunPlanning(obstacle_density, policy_type, log_file);
}
