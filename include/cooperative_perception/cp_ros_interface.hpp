#pragma once
#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_object_kinematics.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "cooperative_perception/msg/intervention.hpp"
#include "cooperative_perception/srv/intervention.hpp"
#include "cooperative_perception/srv/state.hpp"
#include "cooperative_perception/srv/update_perception.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class CPRosInterface: public rclcpp::Node {
public:
    CPRosInterface();

private:

    autoware_auto_perception_msgs::msg::PredictedObjects predicted_objects_;
    geometry_msgs::msg::Pose ego_pose_;
    geometry_msgs::msg::Twist ego_speed_;
    autoware_auto_planning_msgs::msg::Trajectory ego_trajectory_;
    cooperative_perception::msg::Intervention intervention_result_;

private:
    rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr _sub_objects;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _sub_ego_pose;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr _sub_ego_speed;
    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr _sub_ego_traj;
    rclcpp::Subscription<cooperative_perception::msg::Intervention>::SharedPtr _sub_intervention;

    rclcpp::Publisher<cooperative_perception::msg::Intervention>::SharedPtr _pub_intervention;
    rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObject>::SharedPtr _pub_updated_object;

    rclcpp::Service<cooperative_perception::srv::Intervention>::SharedPtr _intervention_service;
    rclcpp::Service<cooperative_perception::srv::State>::SharedPtr _current_state_service;
    rclcpp::Service<cooperative_perception::srv::UpdatePerception>::SharedPtr _update_perception_service;

private:
    void EgoPoseCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg); 
    void EgoSpeedCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg); 
    void EgoTrajectoryCb(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);
    void InterventionCb(const cooperative_perception::msg::Intervention::SharedPtr msg);
    void ObjectsCb(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg);

    void InterventionService(const std::shared_ptr<cooperative_perception::srv::Intervention::Request> request, std::shared_ptr<cooperative_perception::srv::Intervention::Response> response);
    void CurrentStateService(const std::shared_ptr<cooperative_perception::srv::State::Request> request, std::shared_ptr<cooperative_perception::srv::State::Response> response);
    void GetCollisionPointAndRisk(const autoware_auto_planning_msgs::msg::Trajectory &ego_traj, const autoware_auto_perception_msgs::msg::PredictedObjectKinematics &obj_kinematics, double &collision_prob, double &collision_point, int &path_index) const;
    void UpdatePerceptionService(const std::shared_ptr<cooperative_perception::srv::UpdatePerception::Request> request, std::shared_ptr<cooperative_perception::srv::UpdatePerception::Response> response);

};
