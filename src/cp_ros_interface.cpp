#include "cooperative_perception/cp_ros_interface.hpp"

CPRosInterface::CPRosInterface()
    : Node("CPRosInterface")
{
    _sub_objects = this->create_subscription<autoware_auto_perception_msgs::msgs::PredictedObjects>("/objects/predicted_objects", 10, std::bind(&CPRosInterface::ObjectsCb, this, _1));
    _sub_ego_pose = this->create_subscription<geometry_msgs::msgs::PoseWithCovarianceStampled>("/localization/pose_with_covariance", 10, std::bind(&CPRosInterface::EgoPoseCb, this, _1));
    _sub_ego_speed = this->create_subscription<geometry_msgs::msgs::TwistWithCovarianceStamped>("/localization/twist", 10, std::bind(&CPRosInterface::EgoSpeedCb, this, _1));
    _sub_ego_traj = this->create_subscription<autoware_auto_planning_msgs::msgs::Trajectory>("/planning/scenario_planning/trajectory", 10, std::bind(&CPRosInterface::EgoTrajectoryCb, this, _1));
    _sub_intervention = this->create_subscription<cooperative_perception::msgs::InterventionTarget>("/intervention_result", 10, std::bind(&CPRosInterface::InterventionCb, this, _1));

    _pub_updated_objects = this->create_publisher<cooperative_perception::msg::Intervention>("/updated_objects", 10);
    _pub_intervention = this->create_publisher<cooperative_perception::msg::Intervention>("/intervention_target", 10);

    _intervention_service = this->create_service<cooperative_perception::srv::Intervention> ("intervention", &std::bind(&CPRosInterface::InterventionService, this, _1, _2));
    _current_state_service = this->create_service<cooperative_perception::srv::State> ("cp_current_state", &std::bind(&CPRosInterface::CurrentStateService, this, _1, _2));
    _update_perception_service = this->create_service<cooperative_perception::srv::UpdatePerception> ("cp_updated_target", &std::bind(&CPRosInterface::UpdatePerceptionService, this, _1, _2));

}

void CPRosInterface::EgoPoseCb(geometry_msgs::msg::PoseWithCovarianceStamped &msg) 
{
    ego_pose_ = msg.pose.pose;
}

void CPRosInterface::EgoSpeedCb(geometry_msgs::msg::TwistWithCovarianceStamped &msg) 
{
    ego_speed_ = msg.twist.twist;
}

void CPRosInterface::EgoTrajectoryCb(autoware_auto_perception_msgs::msg::Trajectory &msg) 
{
    ego_trajectory_ = msg;
}

void CPRosInterface::InterventionCb(cooperative_perception::msg::InterventionTarget &msg)
{
    intervention_result_ = msg;
}

void CPRosInterface::ObjectsCb(autoware_auto_perception_msgs::msg::PredictedObjects &msg) 
{
    _predicted_objects = msg;
}



void CPRosInterface::InterventionService(const std::shared_ptr<cooperative_perception::srv::Intervention::Request> request, std::shared_ptr<cooperative_perception::srv::Intervention::Responce> responce)
{
    cooperative_perception::msg::Intervention out_msg;
    out_msg.object_id = request->object_id;
    for (const auto &object : _predicted_objects)
    {
        if (object.object_id == out_msg.object_id)
        {
            double collision_prob;
            double collision_point;
            int path_index;
            GetCollisionPointAndRisk(_ego_trajectory, object.kinematics.predicted_paths, collision_prob, collision_point, path_index);

            out_msg.distance = collision_point;
            out_msg.path_index = path_index;
            _pub_action->publish(msg);

            responce->result = true;
        }
    }
    responce->result = false;
}


void CPRosInterface::CurrentStateService(const std::shared_ptr<cooperative_perception::srv::State::Request> request, std::shared_ptr<cooperative_perception::srv::State::Responce> responce)
{
    std::vector<double> distances, probs;
    std::vector<unique_identifier_msgs::msg::UUID> ids;
    for (const auto &object: _predicted_objects)
    {
    /* get collision point and confidence */
        double collision_prob;
        double collision_point;
        int path_index;
        GetCollisionPointAndRisk(_ego_trajectory, object.kinematics.predicted_paths, collision_prob, collision_point, path_index);

        /* no collision point -> ignore it */
        if (collsion_point = 0.0) continue;

        distances.emplace_back(std::int8_t(collision_point));
        probs.emplace_back(collision_prob);
        ids.emplace_back(object.object_id);
    }
    responce->target_pose = distances;
    responce->likelihood = probs;
    responce->ego_speed = ego_speed_.linear.x;
    responce->object_id = ids;
}

void CPRosInterface::GetCollisionPointAndRisk(const autoware_auto_planning_msgs::msg::Trajectory &ego_traj, const autoware_auto_perception_msgs::msg::PredictedPath *obj_paths, float &collision_prob, float &collision_point, int &path_index)
{
    float thres = 3.0; // [m]
    float accumulated_dist = 0.0;
    for (int i=0; i<sizeof(ego_traj.points)/sizeof(autoware_auto_planning_msgs::msg::TrajectoryPoint); ++i)
    {
        /* object position is based on the crossing point */
        geometry_msgs::msg::Pose &ego_traj_pose = ego_traj[i];
        if (i == 0)
        {
            incremental_dist += sqrt(pow(ego_traj_pose.position.x - ego_pose_.position.x) + pow(ego_traj_pose.position.y - ego_pose_.position.y));
        } else 
        {
            incremental_dist += sqrt(pow(ego_traj_pose.position.x - ego_traj[i-1].position.x) + pow(ego_traj_pose.position.y - ego_traj[i-1].position.y));
        }

        /* calculate crossing point */
        for (int j=0; j<sizeof(obj_paths)/sizeof(autoware_auto_perception_msgs::msg::PredictedPath); ++j) 
        {
            for (int k=0; k<sizeof(obj_paths[j].path)/sizeof(geometry_msgs::msg::Pose); ++k)
            {
                geometry_msgs::msg::Pose &obj_pose = obj_paths[j].path[k];

                float dist = sqrt(pow(obj_pose.point.x - ego_traj_pose.point.x) + pow(obj_pose.point.y - ego_traj_pose.point.y));
                if (dist < thres)
                {
                    collision_point = accumulated_dist;
                    collision_prob = obj_paths[j].confidence;
                    path_index = j;
                    return;
                }
            }
        }
    }
}

void CPRosInterface::UpdatePerceptionService(const std::shared_ptr<cooperative_perception::srv::UpdatePerception::Request> request, std::shared_ptr<cooperative_perception::srv::UpdatePerception::Responce> responce)
{
    for (auto &object : _predicted_objects)
    {
        if (object.object_id == request->object_id)
        {
            double collision_prob;
            double collision_point;
            int path_index;
            GetCollisionPointAndRisk(_ego_trajectory, object.kinematics.predicted_paths, collision_prob, collision_point, path_index);
            object.kinematics.predicted_paths[path_index].confidence = request->likelihood;
            responce->result = true;
            _pub_updated_objects->publish(object);
            break;
        }
    }
    responce->result = false;
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CPRosInterface>(argc, argv));
    rclcpp::shutdown();
    return 0;
}
