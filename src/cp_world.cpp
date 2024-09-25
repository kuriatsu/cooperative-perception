#include "cooperative_perception/cp_world.hpp"

CPWorld::CPworld()
    : Node("CPWorldNode")
{

    _intervention_client = this->create_service<cooperative_perception::srv::Intervention>("intervention");
    _current_state_client = this->create_service<cooperative_perception::srv::State>("cp_current_state");
    _synthesize_state_client = this->create_service<cooperative_perception::srv::State>("cp_updated_target");
}

CPWorld::~CPworld() {
    rclcpp::shutdown();
}


State* CPWorld::Initialize() 
{
    return nullptr;
}

bool Connect(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CPWorld>(argc, argv));
}

State* RasWorld::GetCurrentState() 
{
}



State* CPWorld::GetCurrentState() {
    return static_cast<State*>(_pomdp_state);
}


void CPWorld::GetCurrentState(autoware_perception_msgs::msg::PredictedObjects &objects, State* state, std::vector<double> *likelihood_list) 
{

    auto request = std::make_shared<cooperative_perception::srv::State::Request>();
    request->req = true;

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node] Interrupted while waiting for service. Exit");
            return 0;
        } 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world_node] service not available");
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
    } else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node] failed to call service Intervention");
    }
}



bool CPWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {

    std::cout << "execute action" << std::endl;
    CPState::ACT ta_action = _cp_values->getActionAttrib(action);
    int target_idx = _cp_values->getActionTarget(action);
    
    // intervention request
    if (ta_action == CPState::REQUEST) {
        unique_identifier_msgs::msg::UUID target_id = id_idx_list[target_idx];
        if (req_target_history.empty() || req_target_history.back() == req_target_id) {
            _pomdp_state->req_time += Globals::config.time_per_move;
        }
        else {
            _pomdp_state->ego_recog[target_idx] = CPState::RISK;
            _pomdp_state->req_time = Globals::config.time_per_move;
            _pomdp_state->req_target = target_idx;
        }

        req_target_history.emplace_back(req_target_id);

        cooperative_perception::msg::Intervention out_msg;
        msg.object_id = target_id;
        msg.distance = _pomdp_state->risk_pose[target_idx];
        _pub_action->publish(msg);
    }
    
    // NO_ACTION
    else {
        std::cout << "NO_ACTION" << std::endl;
        _pomdp_state->req_time = 0;
        _pomdp_state->req_target = 0;

        req_target_history.emplace_back("none");

        cooperative_perception::msg::Intervention out_msg;
        _pub_action->publish(msg);
    }

    auto request = std::make_shared<cooperative_perception::srv::State::Request>();
    request->req = true;

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node] Interrupted while waiting for service. Exit");
            return 0;
        } 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world_node] service not available");
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        result.get()->obs;
    } else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node] failed to call service Intervention");
    }

    return false;
}


ACT_TYPE CPWorld::MyopicAction() {

    // if intervention requested to the target and can request more
    if (0 < _pomdp_state->req_time && _pomdp_state->req_time < 6 && _pomdp_state->risk_pose[_pomdp_state->req_target] > vehicle_model->getDecelDistance(_pomdp_state->ego_speed, vehicle_model->m_max_decel, 0.0)) {
        return cp_values->getAction(CPState::REQUEST, _pomdp_state->req_target);
    }
    // finish request and change state 
    else if (0 < _pomdp_state->req_time && _pomdp_state->ego_recog[_pomdp_state->req_target] != obs_history.back()) {
        return cp_values->getAction(CPState::RECOG, _pomdp_state->req_target);
    }

    // find request target
    int closest_target = -1, min_dist = 100000;
    for (int i=0; i<_pomdp_state->risk_pose.size(); i++) {
        int is_in_history = std::count(req_target_history.begin(), req_target_history.end(), id_idx_list[i]);
        double request_distance = vehicle_model->getDecelDistance(_pomdp_state->ego_pose, vehicle_model->m_min_decel, vehicle_model->m_safety_margin) + vehicle_model->m_yield_speed * (6.0 - vehicle_model->getDecelTime(_pomdp_state->ego_speed, vehicle_model->m_min_decel)); 
       if (is_in_history == 0 && _pomdp_state->risk_pose[i] > request_distance) {
            if (_pomdp_state->risk_pose[i] < min_dist) {
               min_dist = _pomdp_state->risk_pose[i];
               closest_target = i;
            }
        }
    }

    if (closest_target != -1) {
        return cp_values->getAction(CPState::REQUEST, closest_target);
    }
    else
        return cp_values->getAction(CPState::NO_ACTION, 0);
}

ACT_TYPE CPWorld::EgoisticAction() {
    return cp_values->getAction(CPState::NO_ACTION, 0);
}

void RasWorld::Log(ACT_TYPE action, OBS_TYPE obs) {
    double time;
    Pose ego_pose;
    std::vector<double> other_ego_info;
    std::vector<Risk> log_risks;
    sim->log(time, ego_pose, other_ego_info, log_risks);

    std::string log_action = cp_values->getActionName(action);
    std::string log_obs = cp_values->getObsName(obs);
    std::string log_action_target = "NONE";
    if (log_action != "NO_ACTION") {
        log_action_target = id_idx_list[cp_values->getActionTarget(action)];
    }

    nlohmann::json step_log = {
        {"time", time},
        {"x", ego_pose.x},
        {"y", ego_pose.y},
        {"lane_position", ego_pose.lane_position},
        {"lane", ego_pose.lane},
        {"speed", other_ego_info[0]}, 
        {"accel", other_ego_info[1]},
        {"fuel_consumption", other_ego_info[2]},
        {"action", log_action},
        {"action_target", log_action_target},
        {"obs", log_obs}
    };

    for (const auto& risk : log_risks) {
        nlohmann::json buf = {
            {"id", risk.id},
            {"x", risk.pose.x},
            {"y", risk.pose.y},
            {"lane_position", risk.pose.lane_position},
            {"lane", risk.pose.lane},
            {"prob", risk.risk_prob},
            {"pred", risk.risk_pred},
            {"hidden", risk.risk_hidden}
        };
        step_log["risks"].emplace_back(buf);
    }

    m_log.emplace_back(step_log);
}

