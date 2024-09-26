#pragma once
#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/Trajectory.hpp"
#include "geometry_msgs/msg/PoseWithCovarianceStampled.hpp"
#include "cooperative_perception/msg/Intervention.hpp"
#include "unique_identifier_msgs/msg/UUID.hpp"

class CPRosInterface: public rclcpp::Node {
public:
    CPRosInterface();

private:

    autoware_auto_perception_msgs::msg::PredictedObjects _predicted_objects;
    geometry_msgs::msg::Pose ego_pose_;
    geometry_msgs::msg::Twist ego_speed_;
    autoware_auto_planning_msgs::msg::Trajectory ego_trajectory_;
    cooperative_perception::msg::InterventionTarget intervention_result_;

private:
    rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr _sub_objects;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _sub_ego_pose;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr _sub_ego_speed;
    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr _sub_ego_traj;
    rclcpp::Subscription<cooperative_perception::msg::InterventionTarget>::SharedPtr _sub_intervention;

    rclcpp::Publisher<cooperative_perception::msg::InterventionTarget>::SharedPtr _pub_intervention;
    rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObject>::SharedPtr _pub_updated_object;

    rclcpp::Service<cooperative_perception::srv::Intervention>:SharedPtr _intervention_service;
    rclcpp::Service<cooperative_perception::srv::State>:SharedPtr _current_state_service;
    rclcpp::Service<cooperative_perception::srv::State>:SharedPtr _update_perception_service;

private:
void EgoPoseCb(geometry_msgs::msg::PoseWithCovarianceStamped &msg); 
void EgoSpeedCb(geometry_msgs::msg::TwistWithCovarianceStamped &msg); 
void EgoTrajectoryCb(autoware_auto_perception_msgs::msg::Trajectory &msg);
void InterventionCb(cooperative_perception::msg::InterventionTarget &msg);
void ObjectsCb(autoware_auto_perception_msgs::msg::PredictedObjects &msg);

void InterventionService(const std::shared_ptr<cooperative_perception::srv::Intervention::Request> request, std::shared_ptr<cooperative_perception::srv::Intervention::Responce> responce);
void CurrentStateService(const std::shared_ptr<cooperative_perception::srv::State::Request> request, std::shared_ptr<cooperative_perception::srv::State::Responce> responce);
void GetCollisionPointAndRisk(const autoware_auto_planning_msgs::msg::Trajectory &ego_traj, const autoware_auto_perception_msgs::msg::PredictedPath *obj_paths, float &collision_prob, float &collision_point, int &path_index);
void UpdatePerceptionService(const std::shared_ptr<cooperative_perception::srv::UpdatePerception::Request> request, std::shared_ptr<cooperative_perception::srv::UpdatePerception::Responce> responce);

};
