#pragma once
#include "rclcpp/rclcpp.hpp"

#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_planning_msgs/msg/Trajectory.hpp"
#include "geometry_msgs/msg/PoseWithCovarianceStampled.hpp"
#include "cooperative_perception/msg/Intervention.hpp"
#include "unique_identifier_msgs/msg/UUID.hpp"

class CPRosService: public rclcpp::Node {
public:
    CPRosService();

private:

    autoware_auto_perception_msgs::msg::PredictedObjects _predicted_objects;
    geometry_msgs::msg::Pose _ego_pose;
    geometry_msgs::msg::Twist _ego_speed;
    autoware_planning_msgs::msg::Trajectory _ego_trajectory;
    cooperative_perception::msg::InterventionTarget _intervention_result;
    CPState _cp_state;

private:
    rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr _sub_objects;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _sub_ego_pose;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr _sub_ego_speed;
    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr _sub_ego_traj;

    rclcpp::Subscription<cooperative_perception::msg::InterventionTarget>::SharedPtr _sub_intervention;

    rclcpp::Publisher<cooperative_perception::msg::InterventionTarget>::SharedPtr _pub_intervention;
    rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObject>::SharedPtr _pub_updated_object;

    rclcpp::Service<cooperative_perception::srv::Intervention>:SharedPtr _intervention_service;
    rclcpp::Service<cooperative_perception::srv::State>:SharedPtr _current_state_service;
    rclcpp::Service<cooperative_perception::srv::State>:SharedPtr _update_perception_service;

};
