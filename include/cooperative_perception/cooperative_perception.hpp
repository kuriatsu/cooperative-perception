#pragma once

#include <memory>
#include <despot/planner.h>

#include "cooperative_perception_planner/task_allocation.h"
#include "cooperative_perception_planner/operator_model.h"
#include "cooperative_perception_planner/sumo_interface.h"

#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_planning_msgs/msg/Trajectory.hpp"
#include "geometry_msgs/msg/PoseWithCovarianceStampled.hpp"
#include "cooperative_perception/msg/Intervention.hpp"
#include "unique_identifier_msgs/msg/UUID.hpp"

#include <cmath>


using namespace despot;
using std::placeholders::_1;

class CooperativePerceptionPlanner: public Planner 
{
public:
	CooperativePerceptionPlanner(int argc, char* argv[]);

    // params
    double _delta_t = 2.0;
    double _risk_thresh = 0.5;
    int _planning_horizon = 150;

    // model parameters
    string _policy_type = "DESPOT"; // DESPOT, MYOPIC, EGOISTIC
    
    // pomdp
    Solver *_solver;
    Logger *_logger;
    Belief *_belief;
    Model *_model;
    World *_world;
    
    // models
    OperatorModel *_operator_model;
    VehicleModel *_vehicle_model;
    
    // kinda like enum of action and observation for POMDP  
    // dynamically updated

private:



    
    //
    bool _on_belief_update;


private:
};
