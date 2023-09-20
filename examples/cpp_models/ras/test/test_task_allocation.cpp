#include <despot/planner.h>

#include "task_allocation.h"
#include "nlohmann/json.hpp"
#include <random>

using namespace despot;

class MyPlanner: public Planner {
private:
    std::string log_filename;
    double delta_t;
    double time_per_move;
    std::vector<int> risk_pose;
    std::vector<double> risk_likelihood;

public:
    MyPlanner() {
    }

    MyPlanner(const std::string filename) {
        log_filename = filename;
        std::ifstream f(filename);
        nlohmann::json params = nlohmann::json::parse(f);
        delta_t = params["delta_t"];
        time_per_move = params["time_per_move"];
        risk_pose = static_cast<std::vector<int>>(params["risk_pose"]);
        risk_likelihood = static_cast<std::vector<double>>(params["risk_likelihood"]);
    }

    MyPlanner(const std::string filename, const double delta_t_, const std::vector<int> risk_pose_, const std::vector<double>risk_likelihood_) {
        log_filename = filename;
        delta_t = delta_t_;
        time_per_move = delta_t_;
        risk_pose = risk_pose_;
        risk_likelihood = risk_likelihood_;
    }

    DSPOMDP* InitializeModel(option::Option* options) {
		DSPOMDP* model = new TaskAllocation(delta_t, risk_pose, risk_likelihood);
		// DSPOMDP* model = new TaskAllocation(params["delta_t"], params["risk_pose"], params["risk_likelihood"]);
        return model;
    }

    World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options)
    {
        return InitializePOMDPWorld(world_type, model, options);
    }

    void InitializeDefaultParameters() {
        Globals::config.num_scenarios = 500;
        Globals::config.sim_len = 100;
        Globals::config.search_depth = 100;
        Globals::config.time_per_move = time_per_move;
    }

    std::string ChooseSolver(){
        return "DESPOT";
    }

    int RunEvaluation(int argc, char *argv[]) {

        /* =========================
         * initialize parameters
         * =========================*/
        string solver_type = ChooseSolver();
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

        /* =========================
         * initialize model
         * =========================*/
        DSPOMDP *model = InitializeModel(options);
        assert(model != NULL);

        /* =========================
         * initialize world
         * =========================*/
        World *world = InitializeWorld(world_type, model, options);
        assert(world != NULL);

        /* =========================
         * initialize belief
         * =========================*/
        Belief* belief = model->InitialBelief(world->GetCurrentState(), belief_type);
        assert(belief != NULL);

        /* =========================
         * initialize solver
         * =========================*/
        Solver *solver = InitializeSolver(model, belief, solver_type, options);

        /* =========================
         * initialize logger
         * =========================*/
        Logger *logger = NULL;
        InitializeLogger(logger, options, model, belief, solver, num_runs,
                main_clock_start, world, world_type, time_limit, solver_type);
        //logger->world_seed(world_seed);

        int start_run = 0;

        /* =========================
         * Display parameters
         * =========================*/
        DisplayParameters(options, model);

        /* =========================
         * run evaluation
         * =========================*/
        EvaluationLoop(model, world, belief, belief_type, solver, logger,
                options, main_clock_start, num_runs, start_run);

        //logger->End();

        PrintResult(num_runs, logger, main_clock_start);

        nlohmann::json log;
        try {
            std::ifstream f(log_filename);
            log = nlohmann::json::parse(f);
        }
        catch (...) {
            log["log"] = {};
        }

        nlohmann::json buf;
        buf["delta_t"] = delta_t;
        buf["time_per_move"] = time_per_move;
        buf["risk_pose"] = risk_pose;
        buf["risk_likelihood"] = risk_likelihood;
        buf["total discounted reward"] = logger->AverageDiscountedRoundReward();
        buf["total discounted reward stderr"] = logger->StderrDiscountedRoundReward();
        buf["total undiscounted reward"] = logger->AverageUndiscountedRoundReward();
        buf["total undiscounted reward stderr"] = logger->AverageUndiscountedRoundReward();
        log["log"].emplace_back(buf);

        std::ofstream o(log_filename);
        o << std::setw(4) << log << std::endl;

        return 0;
    }
    };


int main(int argc, char* argv[]) {
    // return MyPlanner(argv[1]).RunEvaluation(argc, argv);
    // return MyPlanner().RunEvaluation(argc, argv);
    std::mt19937 mt{std::random_device{}()};
    std::uniform_int_distribution<int> pose_dist(10, 140);
    std::uniform_real_distribution<double> likelihood_dist(0.0, 1.0);
    std::vector<double> time_step_list = {1.0, 2.0, 3.0, 6.0};
    for (const auto time_step : time_step_list) {
        for (int risk_num=1; risk_num < 8; ++risk_num) {
            for (int round = 0; round < 10; round ++) {
                std::vector<int> risk_pose;
                std::vector<double> risk_likelihood;
                for (int i=0; i<risk_num; i++) {
                    risk_pose.emplace_back(pose_dist(mt));
                    risk_likelihood.emplace_back(likelihood_dist(mt));
                }
                MyPlanner(argv[1], time_step, risk_pose, risk_likelihood).RunEvaluation(argc, argv);
            }
        }
    }
    return 0;
}
