#pragma once
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <std_msgs/msg/string.hpp>

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_object_kinematics.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include "cooperative_perception/msg/intervention.hpp"
#include "cooperative_perception/msg/predicted_object.hpp"
#include "cooperative_perception/srv/intervention.hpp"
#include "cooperative_perception/srv/state.hpp"
#include "cooperative_perception/srv/update_perception.hpp"
#include "cooperative_perception/libgeometry.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

class CPRosInterface: public rclcpp::Node {
public:
    CPRosInterface();
    struct Object {
        autoware_auto_perception_msgs::msg::PredictedObject predicted_object;
        int decay_time;
        double collision_prob;
        double collision_point;
        int collision_path_index; 
    };

private:

    std::map<std::string, Object> objects_; // id, object
    geometry_msgs::msg::Pose ego_pose_;
    geometry_msgs::msg::Twist ego_speed_;
    autoware_auto_planning_msgs::msg::Trajectory ego_trajectory_;
    cooperative_perception::msg::Intervention intervention_result_;

private:
    rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr sub_objects_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_ego_pose_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_ego_speed_;
    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr sub_ego_traj_;
    rclcpp::Subscription<cooperative_perception::msg::Intervention>::SharedPtr sub_intervention_;

    rclcpp::Publisher<cooperative_perception::msg::Intervention>::SharedPtr pub_intervention_;
    rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr pub_objects_;
    rclcpp::Publisher<cooperative_perception::msg::PredictedObject>::SharedPtr pub_cp_object_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_trajectory_;

    rclcpp::Service<cooperative_perception::srv::Intervention>::SharedPtr intervention_service_;
    rclcpp::Service<cooperative_perception::srv::State>::SharedPtr current_state_service_;
    rclcpp::Service<cooperative_perception::srv::UpdatePerception>::SharedPtr update_perception_service_;

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
