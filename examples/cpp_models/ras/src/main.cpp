#include <despot/planner.h>
#include "task_allocation.h"
#include "ras_world.h"
#include "operator_model.h"
#include "sumo_interface.h"
#include <unistd.h>
// #include <boost/program_options.hpp>

using namespace despot;

class MyPlanner: public Planner {
// extern char *optarg;
// extern int optind, opterr, optopt;

private:
    std::vector<double> _perception_range = {50, 150}; // left+right range, forward range
                                                      
    // model parameters
    string _world_type = "simulator";
    string _belief_type = "DEFAULT";
    option::Option *options;
    string _policy_type = "DESPOT"; // DESPOT, MYOPIC, EGOISTIC, REFERENCE
                                    
    // log
    string _log_file = "";
    
    // simulation params
    int _planning_horizon = 150;
    double _risk_thresh = 0.5;
    double _obstacle_density = 0.01; // density 1ppl/m, 0.1=1ppl per 10m, 0.01=1ppl per 100m

    // perception param
    double _perception_acc_ave = 0.75;
    double _perception_acc_dev = 0.1;
 
    // operator_model
    double _min_time = 3.0;
    double _acc_time_min = 0.5;
    double _acc_time_slope = 0.25;
    double _max_acc = 1.0;

    // vehicle model
    double _max_speed = 11.2; // 13.8
    double _yield_speed = 2.8;
    double _max_accel = 0.15 * 9.8;
    double _max_decel = 0.3 * 9.8;
    double _min_decel = 0.2 * 9.8;
    int _safety_margin = 20;

    double _delta_t = 1.0;
    double _time_per_move = 1.0;

public:
    MyPlanner(){

    }

    // models
    OperatorModel *_operator_model = new OperatorModel(_min_time, _acc_time_min, _acc_time_slope, _max_acc);
    VehicleModel *_vehicle_model = new VehicleModel(_max_speed, _yield_speed, _max_accel, _max_decel, _min_decel, _safety_margin, _delta_t);

	DSPOMDP* InitializeModel(option::Option* options) {
		DSPOMDP* model = new TaskAllocation(_planning_horizon, _risk_thresh, _vehicle_model, _operator_model, _delta_t);
		return model;
	}

	World* InitializeWorld(std::string& _world_type, DSPOMDP* model, option::Option* options) {
        RasWorld* ras_world = new RasWorld(_vehicle_model, _operator_model, _delta_t, _obstacle_density, _perception_range, _policy_type, _perception_acc_ave, _perception_acc_dev);
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
        
        for (int i=0; i<_delta_t; i++) {
            ras_world->Step(0);
        }

        if (ras_world->isTerminate()) {
            return true;
        }

        TAState* start_state = static_cast<TAState*>(ras_world->GetCurrentState());
        std::vector<double> likelihood_list = ras_world->GetPerceptionLikelihood();
        ta_model->syncCurrentState(start_state, likelihood_list);

        Belief* belief = ta_model->InitialBelief(start_state, likelihood_list, _belief_type);
        assert(belief != NULL);
        // solver->belief(belief);

        solver = InitializeSolver(model, belief, ChooseSolver(), options);

        ACT_TYPE action;
        if (_policy_type == "MYOPIC")
            action = ras_world->MyopicAction();
        else if (_policy_type == "EGOISTIC" || _policy_type == "REFERENCE")
            action = ras_world->EgoisticAction();
        else {
            std::cout << "get action" << std::endl;
            action = solver->Search().action;
        }

        std::cout << "get obs" << std::endl;
        OBS_TYPE obs;
        bool terminal = ras_world->ExecuteAction(action, obs);
        solver->BeliefUpdate(action, obs);

        std::cout << "update simulator state" << std::endl;
        if (_policy_type == "REFERENCE")
            ras_world->sim->controlEgoVehicle(start_state->risk_pose, start_state->risk_bin);
        else
            ras_world->UpdateState(action, obs, ta_model->getRiskProb(belief));

        ras_world->Log(action, obs);

        return false;
    }

    
    int RunPlanning(int argc, char* argv[]) {

        /* =========================
         * initialize parameters
         * =========================*/
//        using boost::program_options::options_description;
//        using boost::program_options::value;
//        using boost::program_options::variables_map;
//        using boost::program_options::store;
//        using boost::program_options::parse_command_line;
//        using boost::program_options::notify;
//        options_description description("test");
//        description.add_options()
//            ("density", value<double>(), "obstacle density")
//            ("policy", value<std::string>(), "policy")
//            ("log", value<std::string>(), "output log file")
//            ;
//
//        variables_map vm;
//        // store(command_line_parser(split_unix(std::string(*argv))).options(description).run(), vm);
//        store(parse_command_line(argc, argv, description), vm);
//        notify(vm);
//
//        if (vm.count("density"))
//            _obstacle_density = vm["density"].as<double>();
//        if (vm.count("policy"))
//            _policy_type = vm["policy"].as<std::string>();
//        if (vm.count("log"))
//            _log_file = vm["log"].as<std::string>();

         int opt;
         while ((opt = getopt(argc, argv, "d:p:l:")) != -1) {
             if (opt == 'd') 
                 _obstacle_density = std::stof(std::string(optarg));
             else if (opt == 'p')
                 _policy_type = std::string(optarg);
             else if (opt == 'l')
                 _log_file = std::string(optarg);
         }
        argc = 0;
        argv = {};

        bool search_solver;
        int num_runs = 1;
        int time_limit = -1;


        std::string solver_type = ChooseSolver();
        options = InitializeParamers(argc, argv, solver_type,
                    search_solver, num_runs, _world_type, _belief_type, time_limit);
        if(options==NULL)
            return 0;
        clock_t main_clock_start = clock();

        DSPOMDP *model = InitializeModel(options);
        assert(model != NULL);

        World *world = InitializeWorld(_world_type, model, options);
        assert(world != NULL);

        Belief *belief = NULL;
        Solver *solver = NULL;

        Logger *logger = NULL;
        InitializeLogger(logger, options, model, belief, solver, num_runs,
                main_clock_start, world, _world_type, time_limit, solver_type);

        DisplayParameters(options, model);

        logger->InitRound(world->GetCurrentState());
        round_=0; step_=0;
        PlanningLoop(solver, world, model, logger);
        logger->EndRound();

        PrintResult(1, logger, main_clock_start);
        
        std::stringstream ss;
        ss << _policy_type+std::to_string(_obstacle_density)+"_"; 

        if (_log_file.empty()) {
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
            std::stringstream filename_ss{_log_file};
            std::string filename_s, buf_s;
            std::vector<std::string> filename_splitted;
            while (std::getline(filename_ss, buf_s, '_')) {
                filename_splitted.emplace_back(buf_s);
            }
            ss << filename_splitted.back();
        }

        static_cast<RasWorld*>(world)->SaveLog(ss.str());

        std::cout << "#### close simulator ####" << std::endl;
        static_cast<RasWorld*>(world)->Close();

        return 0;
    }
};

int main(int argc, char* argv[]) {
	return MyPlanner().RunPlanning(argc, argv);
}
