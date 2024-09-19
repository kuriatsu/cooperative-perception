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
    
    // models
    OperatorModel *_operator_model;
    VehicleModel *_vehicle_model;

private:
    TAState* _pomdp_state = new TAState(); // save previous state

    // recognition result
    std::vector<std::string> _id_idx_list;
    std::vector<Risk> _perception_targets;

    // for myopic action
    std::vector<std::string> _req_target_history;
    std::vector<OBS_TYPE> _obs_history;

    rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr _pub_updated_objects;
    rclcpp::Publisher<std_msgs:msg::Uuid>::SharedPtr _pub_action;
    rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr _sub_objects;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub_ego_pose;
    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Paths>::SharedPtr _sub_ego_path;

public:
    // kinda like enum of action and observation for POMDP  
    // dynamically updated
    CPValues* _cp_values = new CPValues();
};
