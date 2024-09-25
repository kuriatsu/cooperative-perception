#include "cooperative_perception/cp_server.hpp"

CPRosService::CPRosService()
    : Node("CPRosService")
{
    _sub_objects = this->create_subscription<autoware_perception_msgs::msgs::PredictedObjects>("/objects/predicted_objects", 10, std::bind(&CPRosService::ObjectsCb, this, _1));
    _sub_ego_pose = this->create_subscription<geometry_msgs::msgs::PoseWithCovarianceStampled>("/localization/pose_with_covariance", 10, std::bind(&CPRosService::EgoPoseCb, this, _1));
    _sub_ego_speed = this->create_subscription<geometry_msgs::msgs::TwistWithCovarianceStamped>("/localization/twist", 10, std::bind(&CPRosService::EgoSpeedCb, this, _1));
    _sub_ego_traj = this->create_subscription<autoware_planning_msgs::msgs::Trajectory>("/planning/scenario_planning/trajectory", 10, std::bind(&CPRosService::EgoTrajCb, this, _1));
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

void CPRosService::EgoTrajCb(autoware_perception_msgs::msg::Trajectory &msg) {
    _ego_traj = msg;
}

void CPRosService::InterventionCb(cooperative_perception::msg::InterventionTarget &msg)
{
    _intervention_result = msg;
    // get observation 
    
    OBS_TYPE obs = msg.intervention;
    obs_history.emplace_back(obs);
    cp_values->printObs(obs);

    // find action
    ACT_TYPE action;
    if (msg.action == CPValues::REQUEST) 
    {
        int target_idx;
        for (const auto itr : _id_idx_list)
        {
            if (itr.second == msg.object_id)
            {
                target_idx = itr.key;
                break;
            }
        }
        action = _cp_values->getAction(msg.action, target_idx);
    } else
    {
        action = msg.action;
        return;
    }
}

void CPRosService::ObjectsCb(autoware_perception_msgs::msg::PredictedObjects &msg) {

    _predicted_objects = msg;
    std::cout << "################GetCurrentState##################" << std::endl;

    id_idx_list.clear();
    int target_index = 0;

    // check wether last request target still exists in the perception targets
    bool is_last_req_target_exist = false;

    _pomdp_state->ego_pose = 0;
    _pomdp_state->ego_speed = _ego_speed.linear.x;
    _pomdp_state->ego_recog.clear();
    _pomdp_state->risk_pose.clear();
    _pomdp_state->risk_bin.clear();
    for (int i=0; i<sizeof(objects.objects)/sizeof(autoware_perception_msgs::msg::PredictedObject); i++) 
    {
        autoware_auto_perception_msgs::msg::PredictedObject &object = objects[i];
        float collision_prob;
        float collision_point;
        int path_index;
        GetCollisionPointAndRisk(ego_traj, object.kinematics.predicted_paths, collision_prob, collision_point, path_index);

        if (collsion_point.x == 0.0 and collision_point.y = 0.0) 
        {
            continue;
        }

        id_idx_list[target_index] = object.object_id;
        likelihood_list->emplace_back(collision_prob);

        _pomdp_state->risk_pose.emplace_back(collision_point);
        _pomdp_state->ego_recog.emplace_back(collision_prob>_risk_thresh);
        _pomdp_state->risk_bin.emplace_back(collision_prob>_risk_thresh);
        
        if (req_target_history.size() > 0 && req_target_history.back() == object.object_id) {
            is_last_req_target_exist = true;
            _pomdp_state->req_target = target_index; 
        }

        std::cout << 
            "id :" << object.object_id << "\n" <<
            "distance :" << collision_point << "\n" <<
            "prob :" << collision_prob << "\n" <<
            std::endl;
        target_index++;
    }


    // check last request and update request time
    if (!is_last_req_target_exist) 
    {
        _pomdp_state->req_time = 0;
    } else 
    {
        std::cout << "req target found again, id: " << req_target_history.back() << ", idx: " << _pomdp_state->req_target << std::endl;
    }

    cp_values = new CPState(_pomdp_state->risk_pose.size());

    state = static_cast<State*>(_pomdp_state);
}

void CPWorld::GetCollisionPointAndRisk(const autoware_auto_planning_msgs::msg::Trajectory &ego_traj, const autoware_auto_perception_msgs::msg::PredictedPath *obj_paths, float &collision_prob, float &collision_point, int &path_index) 
{
    float thres = 3.0; // [m]
    float accumulated_dist = 0.0;
    for (int i=0; i<sizeof(ego_traj.points)/sizeof(autoware_planning_msgs::msg::TrajectoryPoint); ++i)
    {
        /* object position is based on the crossing point */
        geometry_msgs::msg::Pose &ego_traj_pose = ego_traj[i];
        if (i == 0)
        {
            incremental_dist += sqrt(pow(ego_traj_pose.position.x - _ego_pose.position.x) + pow(ego_traj_pose.position.y - _ego_pose.position.y));
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
}

void CPRosService::SynthesizeService(const std::shared_ptr<cooperative_perception::srv::Object::Request> request, std::shared_ptr<cooperative_perception::srv::Object::Responce> responce)
{
}


int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CPRosService>(argc, argv));
