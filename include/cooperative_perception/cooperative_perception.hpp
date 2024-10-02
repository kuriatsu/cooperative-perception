#pragma once

#include <memory>
#include <despot/planner.h>

#include "cooperative_perception/cp_pomdp.hpp"
#include "cooperative_perception/cp_world.hpp"
#include "cooperative_perception/operator_model.hpp"
#include "cooperative_perception/vehicle_model.hpp"
#include "cooperative_perception/modelbase_planner.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "cooperative_perception/msg/intervention.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include <cmath>


using namespace despot;
using std::placeholders::_1;

class CooperativePerception: public Planner 
{
public:
	CooperativePerception();
    int RunPlanning(int argc, char* argv[]);

private:
    // params
    double delta_t_ = 2.0;
    double risk_thresh_ = 0.5;
    int planning_horizon_ = 150;

    // model parameters
    string policy_type_ = "DESPOT"; // DESPOT, MYOPIC, EGOISTIC
    string belief_type_ = "DEFAULT";
    
    // pomdp
    option::Option *options_;
    
    // models
    OperatorModel *operator_model_;
    VehicleModel *vehicle_model_;
    
private:
    void PlanningLoop(Solver*& solver, World* world, DSPOMDP* model, Logger* logger);
    bool RunStep(Solver* solver, World* world, DSPOMDP* model, Logger* logger); 
    void InitializeDefaultParameters(); 
    Solver* CPInitializeSolver(DSPOMDP *model, Belief *belief, World *world);
    std::string ChooseSolver();
    DSPOMDP* InitializeModel(option::Option* options);
    CPPOMDP* InitializeModel (State* state);
    World* InitializeWorld(int argc, char* argv[], std::string& world_type, DSPOMDP* model, option::Option* options);
    World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options);

};
