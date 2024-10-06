#include "cooperative_perception/cp_ros_interface.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

CPRosInterface::CPRosInterface()
    : Node("CPRosInterface")
{
    sub_objects_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>("/perception/object_recognition/objects", 10, std::bind(&CPRosInterface::ObjectsCb, this, _1));
    sub_ego_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/pose_with_covariance", 10, std::bind(&CPRosInterface::EgoPoseCb, this, _1));
    sub_ego_speed_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/localization/twist_estimator/twist_with_covariance", 10, std::bind(&CPRosInterface::EgoSpeedCb, this, _1));
    sub_ego_traj_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>("/planning/scenario_planning/trajectory", 10, std::bind(&CPRosInterface::EgoTrajectoryCb, this, _1));
    sub_intervention_ = this->create_subscription<cooperative_perception::msg::Intervention>("/cooperative_perception/intervention_result", 10, std::bind(&CPRosInterface::InterventionCb, this, _1));

    pub_objects_ = this->create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>("/perception/object_recognition/objects_cooperative_perception", 10);
    pub_cp_object_ = this->create_publisher<cooperative_perception::msg::PredictedObject>("/cooperative_perception/object_ue", 10);
    pub_intervention_ = this->create_publisher<cooperative_perception::msg::Intervention>("/cooperative_perception/intervention_request", 10);
    pub_trajectory_ = this->create_publisher<nav_msgs::msg::Path>("/cooperative_perception/trajectory_path", 10);

    intervention_service_ = this->create_service<cooperative_perception::srv::Intervention> ("/cooperative_perception/intervention", std::bind(&CPRosInterface::InterventionService, this, _1, _2));
    current_state_service_ = this->create_service<cooperative_perception::srv::State> ("/cooperative_perception/cp_current_state", std::bind(&CPRosInterface::CurrentStateService, this, _1, _2));
    update_perception_service_ = this->create_service<cooperative_perception::srv::UpdatePerception> ("/cooperative_perception/cp_updated_target", std::bind(&CPRosInterface::UpdatePerceptionService, this, _1, _2));

}

void CPRosInterface::EgoPoseCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
{
    // std::cout << "get pose" << std::endl;
    ego_pose_ = (*msg).pose.pose;
}

void CPRosInterface::EgoSpeedCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) 
{
    // std::cout << "get speed " << std::endl;
    ego_speed_ = (*msg).twist.twist;
}

void CPRosInterface::EgoTrajectoryCb(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) 
{
    // std::cout << "get trajectory" << std::endl;
    ego_trajectory_ = *msg;

    /* join trajectory for rclUE */
    nav_msgs::msg::Path path;
    path.header = msg->header;
    for (const auto &point : msg->points) {
        geometry_msgs::msg::PoseStamped buf_pose;
        buf_pose.pose = point.pose;
        path.poses.emplace_back(buf_pose);
    }
    pub_trajectory_->publish(path);
}

void CPRosInterface::InterventionCb(const cooperative_perception::msg::Intervention::SharedPtr msg)
{
    intervention_result_ = *msg;
}

void CPRosInterface::ObjectsCb(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg) 
{

    /* store and manage objects */
    for (const auto &msg_obj : msg->objects) {
        std::string object_id = despot::CPRosTools().ConvertUUIDtoIntString(msg_obj.object_id.uuid);

        /* new object */
        if (objects_.count(object_id) == 0) {
            Object buf_obj;
            buf_obj.predicted_object = msg_obj;
            buf_obj.decay_time = 0;
            GetCollisionPointAndRisk(ego_trajectory_, msg_obj.kinematics, buf_obj.collision_prob, buf_obj.collision_point, buf_obj.collision_path_index);
            objects_[object_id] = buf_obj;
        }
        /* update object info */
        else {
            Object &buf_obj = objects_[object_id];
            buf_obj.predicted_object = msg_obj;
            double collision_prob;
            GetCollisionPointAndRisk(ego_trajectory_, msg_obj.kinematics, collision_prob, buf_obj.collision_point, buf_obj.collision_path_index);
        }
    }

    /* publish objects for rclUE */
    autoware_auto_perception_msgs::msg::PredictedObjects out_auto_msg;

    for (auto itr = objects_.begin(), end = objects_.end() ; itr != end;) {
        /* update decay time */
        itr->second.decay_time++;
        if (itr->second.decay_time > 5.0) {
            itr = objects_.erase(itr);
            continue;
        }

        /* autoware perception msg */
        autoware_auto_perception_msgs::msg::PredictedObject auto_obj = itr->second.predicted_object;
        auto_obj.kinematics.predicted_paths[itr->second.collision_path_index].confidence = itr->second.collision_prob;
        out_auto_msg.objects.emplace_back(auto_obj);

        /* pub rclre object msgs */
        cooperative_perception::msg::PredictedObject out_cp_msg;
        out_cp_msg.object_id = itr->second.predicted_object.object_id;
        out_cp_msg.existence_probability = itr->second.predicted_object.existence_probability;
        out_cp_msg.pose = itr->second.predicted_object.kinematics.initial_pose_with_covariance.pose;
        out_cp_msg.dimension = itr->second.predicted_object.shape.dimensions;
        for (const auto& classification : itr->second.predicted_object.classification) {
            out_cp_msg.classifications.emplace_back(classification.label);
            out_cp_msg.classification_confidences.emplace_back(classification.probability);
        }

        for (const auto& path : itr->second.predicted_object.kinematics.predicted_paths) {
            out_cp_msg.pathes.insert(out_cp_msg.pathes.end(), path.path.begin(), path.path.end());
            geometry_msgs::msg::Pose separator;
            out_cp_msg.pathes.emplace_back(separator);
            out_cp_msg.path_confidences.emplace_back(path.confidence);
        }
        out_cp_msg.path_confidences[itr->second.collision_path_index] = itr->second.collision_prob;
        pub_cp_object_->publish(out_cp_msg);

        ++itr;
    }

    pub_objects_->publish(out_auto_msg);
}



void CPRosInterface::InterventionService(const std::shared_ptr<cooperative_perception::srv::Intervention::Request> request, std::shared_ptr<cooperative_perception::srv::Intervention::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "[InterventionService] sending intervention request");
    std::string object_id = despot::CPRosTools().ConvertUUIDtoIntString(request->object_id.uuid);

    if (objects_.count(object_id) != 0) {
        cooperative_perception::msg::Intervention out_msg;
        out_msg.object_id = request->object_id;
        out_msg.distance = objects_[object_id].collision_point;
        out_msg.path_index = objects_[object_id].collision_path_index;
        pub_intervention_->publish(out_msg);
    }

    RCLCPP_INFO(this->get_logger(), "[InterventionService] sending intervention result");
    /*TODO change id to intervention result
     */
    response->object_id = request->object_id;
    // response->object_id = intervention_result_.object_id;
    response->result = intervention_result_.intervention;
}


void CPRosInterface::CurrentStateService(const std::shared_ptr<cooperative_perception::srv::State::Request> request, std::shared_ptr<cooperative_perception::srv::State::Response> response)
{
    if (!request->request) return;
    // RCLCPP_INFO(this->get_logger(), "[CurrentStateService] creating current state");

    std::vector<int> distances; 
    std::vector<double> probs;
    std::vector<unique_identifier_msgs::msg::UUID> ids;
    std::vector<std_msgs::msg::String> types;

    for (const auto &obj: objects_)
    {
        std::stringstream ss;
        ss << "[CurrentStateService] creating current state of " 
           << obj.first;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        /* no collision point -> ignore it */
        if (obj.second.collision_point == 0.0) continue;

        distances.emplace_back(int(obj.second.collision_point));
        probs.emplace_back(obj.second.collision_prob);
        ids.emplace_back(obj.second.predicted_object.object_id);
        std_msgs::msg::String buf_type;
        buf_type.data = "hard";
        // buf_type.data = std::string(obj.second.predicted_object.classification[0].label);
        types.emplace_back(buf_type);
    }

    response->risk_pose = distances;
    response->likelihood = probs;
    response->ego_speed = ego_speed_.linear.x;
    response->object_id = ids;
    response->type = types;
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
    RCLCPP_INFO(this->get_logger(), "[UpdatePerceptionService] sending updated perception");
    std::string object_id = despot::CPRosTools().ConvertUUIDtoIntString(request->object_id.uuid);

    if (objects_.count(object_id) != 0) {
        objects_[object_id].collision_prob = request->likelihood;
        response->result = true;
        return;
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
