#include "cooperative_perception/cp_ros_interface.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

CPRosInterface::CPRosInterface()
    : Node("CPRosInterface")
{
    _sub_objects = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>("/objects/predicted_objects", 10, std::bind(&CPRosInterface::ObjectsCb, this, _1));
    _sub_ego_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/pose_with_covariance", 10, std::bind(&CPRosInterface::EgoPoseCb, this, _1));
    _sub_ego_speed = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/localization/twist", 10, std::bind(&CPRosInterface::EgoSpeedCb, this, _1));
    _sub_ego_traj = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>("/planning/scenario_planning/trajectory", 10, std::bind(&CPRosInterface::EgoTrajectoryCb, this, _1));
    _sub_intervention = this->create_subscription<cooperative_perception::msg::Intervention>("/intervention_result", 10, std::bind(&CPRosInterface::InterventionCb, this, _1));

    _pub_updated_object = this->create_publisher<autoware_auto_perception_msgs::msg::PredictedObject>("/updated_objects", 10);
    _pub_intervention = this->create_publisher<cooperative_perception::msg::Intervention>("/intervention_target", 10);

    _intervention_service = this->create_service<cooperative_perception::srv::Intervention> ("intervention", std::bind(&CPRosInterface::InterventionService, this, _1, _2));
    _current_state_service = this->create_service<cooperative_perception::srv::State> ("cp_current_state", std::bind(&CPRosInterface::CurrentStateService, this, _1, _2));
    _update_perception_service = this->create_service<cooperative_perception::srv::UpdatePerception> ("cp_updated_target", std::bind(&CPRosInterface::UpdatePerceptionService, this, _1, _2));

}

void CPRosInterface::EgoPoseCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
{
    ego_pose_ = (*msg).pose.pose;
}

void CPRosInterface::EgoSpeedCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) 
{
    ego_speed_ = (*msg).twist.twist;
}

void CPRosInterface::EgoTrajectoryCb(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) 
{
    ego_trajectory_ = *msg;
}

void CPRosInterface::InterventionCb(const cooperative_perception::msg::Intervention::SharedPtr msg)
{
    intervention_result_ = *msg;
}

void CPRosInterface::ObjectsCb(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg) 
{
    predicted_objects_ = *msg;
}



void CPRosInterface::InterventionService(const std::shared_ptr<cooperative_perception::srv::Intervention::Request> request, std::shared_ptr<cooperative_perception::srv::Intervention::Response> response)
{
    cooperative_perception::msg::Intervention out_msg;
    out_msg.object_id = request->object_id;
    for (const auto &object : predicted_objects_.objects)
    {
        if (object.object_id == out_msg.object_id)
        {
            double collision_prob;
            double collision_point;
            int path_index;
            GetCollisionPointAndRisk(ego_trajectory_, object.kinematics, collision_prob, collision_point, path_index);

            out_msg.distance = collision_point;
            out_msg.path_index = path_index;
            _pub_intervention->publish(out_msg);

            response->result = true;
        }
    }
    response->result = false;
}


void CPRosInterface::CurrentStateService(const std::shared_ptr<cooperative_perception::srv::State::Request> request, std::shared_ptr<cooperative_perception::srv::State::Response> response)
{
    std::vector<int> distances; 
    std::vector<double> probs;
    std::vector<unique_identifier_msgs::msg::UUID> ids;
    for (const auto &object: predicted_objects_.objects)
    {
    /* get collision point and confidence */
        double collision_prob;
        double collision_point;
        int path_index;
        GetCollisionPointAndRisk(ego_trajectory_, object.kinematics, collision_prob, collision_point, path_index);

        /* no collision point -> ignore it */
        if (collision_point == 0.0) continue;

        distances.emplace_back(int(collision_point));
        probs.emplace_back(collision_prob);
        ids.emplace_back(object.object_id);
    }
    response->risk_pose = distances;
    response->likelihood = probs;
    response->ego_speed = ego_speed_.linear.x;
    response->object_id = ids;
}

void CPRosInterface::GetCollisionPointAndRisk(const autoware_auto_planning_msgs::msg::Trajectory &ego_traj, const autoware_auto_perception_msgs::msg::PredictedObjectKinematics &obj_kinematics, double &collision_prob, double &collision_point, int &path_index) const
{
    float thres = 3.0; // [m]
    float accumulated_dist = 0.0;
    for (int i=0; i<int(sizeof(ego_traj.points)/sizeof(autoware_auto_planning_msgs::msg::TrajectoryPoint)); ++i)
    {
        /* object position is crossing point to ego vehicle trajectory
         * get distance from trajectory */
        const geometry_msgs::msg::Pose &ego_traj_pose = ego_traj.points[i].pose;
        if (i == 0) {
            accumulated_dist += sqrt(pow(ego_traj_pose.position.x - ego_pose_.position.x, 2) + pow(ego_traj_pose.position.y - ego_pose_.position.y, 2));
        } 
        else {
            accumulated_dist += sqrt(pow(ego_traj_pose.position.x - ego_traj.points[i-1].pose.position.x, 2) + pow(ego_traj_pose.position.y - ego_traj.points[i-1].pose.position.y, 2));
        }

        /* calculate crossing point */
        for (int j=0; j<int(sizeof(obj_kinematics.predicted_paths)/sizeof(autoware_auto_perception_msgs::msg::PredictedPath)); ++j) {
            for (int k=0; k<int(sizeof(obj_kinematics.predicted_paths[j].path)/sizeof(geometry_msgs::msg::Pose)); ++k) {
                const geometry_msgs::msg::Pose &obj_pose = obj_kinematics.predicted_paths[j].path[k];

                float dist = sqrt(pow(obj_pose.position.x - ego_traj_pose.position.x, 2) + pow(obj_pose.position.y - ego_traj_pose.position.y, 2));
                if (dist < thres) {
                    collision_point = accumulated_dist;
                    collision_prob = obj_kinematics.predicted_paths[j].confidence;
                    path_index = j;
                    return;
                }
            }
        }
    }
}

void CPRosInterface::UpdatePerceptionService(const std::shared_ptr<cooperative_perception::srv::UpdatePerception::Request> request, std::shared_ptr<cooperative_perception::srv::UpdatePerception::Response> response)
{
    for (auto &object : predicted_objects_.objects)
    {
        if (object.object_id == request->object_id)
        {
            double collision_prob;
            double collision_point;
            int path_index;
            GetCollisionPointAndRisk(ego_trajectory_, object.kinematics, collision_prob, collision_point, path_index);
            object.kinematics.predicted_paths[path_index].confidence = request->likelihood;
            response->result = true;
            _pub_updated_object->publish(object);
            break;
        }
    }
    response->result = false;
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CPRosInterface>());
    rclcpp::shutdown();
    return 0;
}
