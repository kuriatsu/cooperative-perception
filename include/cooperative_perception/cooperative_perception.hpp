#pragma once

#include <memory>
#include <despot/planner.h>
#include "rclcpp/rclcpp.hpp"

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
    World *_world;
    
    // models
    OperatorModel *_operator_model;
    VehicleModel *_vehicle_model;
    
    // kinda like enum of action and observation for POMDP  
    // dynamically updated
    std::shared_ptr<CPValues> _cp_values = std::make_shared<CPValues>();

private:
    std::shared_ptr<CPState> _pomdp_state = std::make_shared<CPState>(); // save previous state
    autoware_auto_perception_msgs::msg::PredictedObjects _predicted_objects;

    // recognition result
    std::map<std::int8_t, unique_identifier_msgs::msg::UUID> _id_idx_list;
    std::vector<Risk> _perception_targets;

    // for myopic action
    std::vector<unique_identifier_msgs::msg::UUID> _req_target_history;
    std::vector<OBS_TYPE> _obs_history;

    // ros msg
    geometry_msgs::msg::Pose _ego_pose;
    geometry_msgs::msg::Twist _ego_speed;
    autoware_planning_msgs::msg::Trajectory _ego_traj;
    
    //
    bool _on_belief_update;


private:
    rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr _sub_objects;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _sub_ego_pose;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr _sub_ego_speed;
    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr _sub_ego_traj;
    rclcpp::Subscription<cooperative_perception::msg::InterventionTarget>::SharedPtr _sub_intervention;
    rclcpp::Publisher<cooperative_perception::msg::InterventionTarget>::SharedPtr _pub_action;
    rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObject>::SharedPtr _pub_updated_object;
};
