#include "cooperative_perception/cp_server.hpp"

CPRosService::CPRosService()
    : Node("CPRosService")
{
    _sub_objects = this->create_subscription<autoware_perception_msgs::msgs::PredictedObjects>("/objects/predicted_objects", 10, std::bind(&CPRosService::ObjectsCb, this, _1));
    _sub_ego_pose = this->create_subscription<geometry_msgs::msgs::PoseWithCovarianceStampled>("/localization/pose_with_covariance", 10, std::bind(&CPRosService::EgoPoseCb, this, _1));
    _sub_ego_speed = this->create_subscription<geometry_msgs::msgs::TwistWithCovarianceStamped>("/localization/twist", 10, std::bind(&CPRosService::EgoSpeedCb, this, _1));
    _sub_ego_traj = this->create_subscription<autoware_planning_msgs::msgs::Trajectory>("/planning/scenario_planning/trajectory", 10, std::bind(&CPRosService::EgoTrajectoryCb, this, _1));
    _sub_intervention = this->create_subscription<cooperative_perception::msgs::InterventionTarget>("/intervention_result", 10, std::bind(&CPRosService::InterventionCb, this, _1));

    _pub_updated_objects = this->create_publisher<cooperative_perception::msg::Intervention>("/updated_objects", 10);
    _pub_intervention = this->create_publisher<cooperative_perception::msg::Intervention>("/intervention_target", 10);

    _intervention_service = this->create_service<cooperative_perception::srv::Intervention> ("intervention", &std::bind(&CPRosService::InterventionService, this, _1, _2));
    _current_state_service = this->create_service<cooperative_perception::srv::State> ("cp_current_state", &std::bind(&CPRosService::CurrentStateService, this, _1, _2));
    _synthesize_state_service = this->create_service<cooperative_perception::srv::State> ("cp_updated_target", &std::bind(&CPRosService::CurrentStateService, this, _1, _2));

}

void CPRosService::EgoPoseCb(geometry_msgs::msg::PoseWithCovarianceStamped &msg) 
{
    _ego_pose = msg.pose.pose;
}

void CPRosService::EgoSpeedCb(geometry_msgs::msg::TwistWithCovarianceStamped &msg) 
{
    _ego_speed = msg.twist.twist;
}

void CPRosService::EgoTrajectoryCb(autoware_perception_msgs::msg::Trajectory &msg) 
{
    _ego_trajectory = msg;
}

void CPRosService::InterventionCb(cooperative_perception::msg::InterventionTarget &msg)
{
    _intervention_result = msg;
}

void CPRosService::ObjectsCb(autoware_perception_msgs::msg::PredictedObjects &msg) 
{
    _predicted_objects = msg;
}



void CPRosService::InterventionService(const std::shared_ptr<cooperative_perception::srv::Intervention::Request> request, std::shared_ptr<cooperative_perception::srv::Intervention::Responce> responce)
{
    cooperative_perception::msg::Intervention out_msg;
    out_msg.object_id = request->object_id;
    out_msg.distance = _pomdp_state->risk_pose[target_idx];
    _pub_action->publish(msg);
    responce->result = true;
}


void CPRosService::CurrentStateService(const std::shared_ptr<cooperative_perception::srv::State::Request> request, std::shared_ptr<cooperative_perception::srv::State::Responce> responce)
{
    responce->ego_speed = _ego_speed;
    responce->ego_trajectory = _ego_trajectory;
    responce->objects = _predicted_objects;
}

void CPRosService::SynthesizeService(const std::shared_ptr<cooperative_perception::srv::Object::Request> request, std::shared_ptr<cooperative_perception::srv::Object::Responce> responce)
{
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CPRosService>(argc, argv));
    rclcpp::shutdown();
    return 0;
}
