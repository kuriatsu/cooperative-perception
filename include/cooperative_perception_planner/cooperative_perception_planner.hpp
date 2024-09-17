#pragma once

#include <memory>
#include <despot/planner.h>
#include "rclcpp/rclcpp.hpp"

#include "cooperative_perception_planner/task_allocation.h"
#include "cooperative_perception_planner/operator_model.h"
#include "cooperative_perception_planner/sumo_interface.h"

#include "autoware_perception_msgs/msg/predicted_objects_UE.hpp"


using namespace despot;
using std::placeholders::_1;

class CooperativePerceptionPlanner: public Planner, public rclcpp::Node 
{
public:
	CooperativePerceptionPlanner(){
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
    double max_decel = 0.3 * 9.8;
    double min_decel = 0.2 * 9.8;
    int safety_margin = 5;

    double delta_t = 2.0;

    // sim model
    double obstacle_density = 0.01; // density 1ppl/m, 0.1=1ppl per 10m, 0.01=1ppl per 100m
    std::vector<double> perception_range = {50, 150}; // left+right range, forward range

    // model parameters
    string world_type = "simulator";
    string belief_type = "DEFAULT";
    string policy_type = "DESPOT"; // DESPOT, MYOPIC, EGOISTIC
    option::Option *options;

    // models
    OperatorModel *operator_model = new OperatorModel(min_time, acc_time_min, acc_time_slope);
    VehicleModel *vehicle_model = new VehicleModel(max_speed, yield_speed, max_accel, max_decel, min_decel, safety_margin, delta_t);

    Solver *solver = NULL;
    World *world = InitializeWorld(world_type, model, options);
    assert(world != NULL);

    options = InitializeParamers(argc, argv, solver_type,
                search_solver, num_runs, world_type, belief_type, time_limit);
    if(options==NULL)
        return 0;
    clock_t main_clock_start = clock();
    DSPOMDP *model = InitializeModel(options);
    assert(model != NULL);

    Belief *belief = NULL;

    Logger *logger = NULL;
    InitializeLogger(logger, options, model, belief, solver, num_runs,
            main_clock_start, world, world_type, time_limit, solver_type);

    DisplayParameters(options, model);

    logger->InitRound(world->GetCurrentState());
    round_=0; step_=0;

    DSPOMDP* InitializeModel(option::Option* options);
    World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options);
    void InitializeDefaultParameters();
    std::string ChooseSolver();
    void PlanningLoop(Solver*& solver, World* world, DSPOMDP* model, Logger* logger);
    bool RunStep(Solver* solver, World* world, DSPOMDP* model, Logger* logger);
    int RunPlanning(int argc, char *argv[]);

};
