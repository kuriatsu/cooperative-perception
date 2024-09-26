#include "cooperative_perception/cp_world.hpp"

CPWorld::CPworld()
    : Node("CPWorldNode")
{
    intervention_client_ = this->create_service<cooperative_perception::srv::Intervention>("intervention");
    current_state_client_ = this->create_service<cooperative_perception::srv::State>("cp_current_state");
    update_perception_client_ = this->create_service<cooperative_perception::srv::UpdatePerception>("cp_updated_target");
}

CPWorld::~CPworld() {
    rclcpp::shutdown();
}


State* CPWorld::Initialize() 
{
    return nullptr;
}

bool CPWorld::Connect(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CPWorld>(argc, argv));
}


State* CPWorld::GetCurrentState() {
    return static_cast<State*>(pomdp_state_);
}


void CPWorld::GetCurrentState(State &state, std::vector<double> &likelihood_list) 
{

    /* request to service */
    auto request = std::make_shared<cooperative_perception::srv::State::Request>();
    request->request = true;

    while (!current_state_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node] Interrupted while waiting for service. Exit");
            return 0;
        } 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world_node] service not available");
    }

    auto result = current_state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node] failed to call service Intervention");
        return;
    }

    /* start making current state*/
    // check wether last request target still exists in the perception targets
    bool is_last_req_target_exist = false;

    _cp_state->risk_pose.clear();
    _cp_state->ego_recog.clear();
    _cp_state->risk_bin.clear();
    _cp_state->ego_pose = 0;
    _cp_state->ego_speed = result.get()->ego_speed;
    _cp_state->risk_pose = result.get()->target_pose;

    id_idx_list_.clear();

    for (int i=0; i<sizeof(result.get()->ids)/sizeof(unique_identifier_msgs::msg::UUID); i++) 
    {
        id_idx_list_[i] = result.get()->object_id[i];
        double &likelihood = result.get()->likelihood[i];
        likelihood_list.emplace_back(likelihood);

        _cp_state->ego_recog.emplace_back(likelihood>_risk_thresh);
        _cp_state->risk_bin.emplace_back(likelihood>_risk_thresh);

        /* is this request target (index can change at each time step) */
        if (req_taret_history_.size() > 0 && req_taret_history_.back() == result.get()->object_id[i]) {
            is_last_req_target_exist = true;
            _cp_state->req_target = target_index; 
        }


        std::cout << 
            "id :" << object.object_id << "\n" <<
            "distance :" << collision_point << "\n" <<
            "prob :" << collision_prob << "\n" <<
            std::endl;
    }


    // check last request and update request time
    if (!is_last_req_target_exist) 
    {
        _cp_state->req_time = 0;
    } else 
    {
        std::cout << "req target found again, id: " << req_target_history.back() << ", idx: " << _cp_state->req_target << std::endl;
    }

    cp_values_ = new CPState(_cp_state->risk_pose.size());

    state = static_cast<State*>(_cp_state);
}




bool CPWorld::ExecuteAction(ACT_TYPE &action, OBS_TYPE& obs) {

    std::cout << "execute action" << std::endl;

    /* request to service */
    auto request = std::make_shared<cooperative_perception::srv::Intervention::Request>();
    
    // intervention request
    if (cp_values_->getActionAttrib(action) == CPState::REQUEST) {
        int req_target_idx = cp_values_->getActionTarget(action);
        unique_identifier_msgs::msg::UUID req_target_id = id_idx_list[req_target_idx];

        /* check intervention request */
        /* keep request to the same target */
        if (req_target_history.empty() || req_target_history.back() == req_target_id) {
            _cp_state->req_time += Globals::config.time_per_move;
        }
        else {
            _cp_state->ego_recog[req_target_idx] = CPState::RISK;
            _cp_state->req_time = Globals::config.time_per_move;
            _cp_state->req_target = req_target_idx;
        }

        req_taret_history_.emplace_back(req_target_id);

        request->action = CPValues::REQUEST;
        request->object_id = req_target_id;

    } else 
    {
        std::cout << "NO_ACTION" << std::endl;
        _cp_state->req_time = 0;
        _cp_state->req_target = 0;

        req_taret_history_.emplace_back("none");

        request->action = CPValues::NO_ACTION;
        request->object_id = 0;
    }

    /* throw request */
    while (!intervention_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node] Interrupted while waiting for service. Exit");
            return 0;
        } 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world_node] service not available");
    }

    /* send request */
    auto result = intervention_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node] failed to call service Intervention");
        return;
    }

    /* process request result (observation) */
    auto intervention_target_id = result.get()->object_id;
    std::int8_t intervention_result = result.get()->result;
    for (const auto &itr : id_idx_list_) 
    {
        if (itr.second == intervention_target_id)
        {
            /* intervention result is the result of action at last time step */
            action = cp_values_->getAction(CPValues::REQUEST, itr.first);
            obs = intervention_result;
            break;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world_node] intervention target no longer exist");
    }

    obs_history_.emplace_back(obs);
    cp_values->printObs(obs);
}


void CPWorld::UpdatePerception(const ACT_TYPE &action, const OBS_TYPE &obs, const std::vector<double> &risk_probs)
{
    std::int8_t target_index = cp_values_->getActionTarget(action);
    unique_identifier_msgs::msg::UUID target_id = ;

    auto request = std::make_shared<cooperative_perception::srv::UpdatePerception::Request>();
    request->object_id = id_idx_list_[target_index];
    request->likelihood = risk_probs[target_index];

    /* throw request */
    while (!update_perception_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node::update_perception_client] Interrupted while waiting for service. Exit");
            return;
        } 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world_node::update_perception_client] service not available");
    }

    /* send request */
    auto result = update_perception_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node::update_perception_client] failed to call service Intervention");
        return;
    }
}


ACT_TYPE CPWorld::MyopicAction() {

    // if intervention requested to the target and can request more
    if (0 < pomdp_state_->req_time && pomdp_state_->req_time < 6 && pomdp_state_->risk_pose[pomdp_state_->req_target] > vehicle_model->getDecelDistance(pomdp_state_->ego_speed, vehicle_model->m_max_decel, 0.0)) {
        return cp_values->getAction(CPState::REQUEST, pomdp_state_->req_target);
    }
    // finish request and change state 
    else if (0 < pomdp_state_->req_time && pomdp_state_->ego_recog[pomdp_state_->req_target] != obs_history_.back()) {
        return cp_values->getAction(CPState::RECOG, pomdp_state_->req_target);
    }

    // find request target
    int closest_target = -1, min_dist = 100000;
    for (int i=0; i<pomdp_state_->risk_pose.size(); i++) {
        int is_in_history = std::count(req_target_history.begin(), req_target_history.end(), id_idx_list[i]);
        double request_distance = vehicle_model->getDecelDistance(pomdp_state_->ego_pose, vehicle_model->m_min_decel, vehicle_model->m_safety_margin) + vehicle_model->m_yield_speed * (6.0 - vehicle_model->getDecelTime(pomdp_state_->ego_speed, vehicle_model->m_min_decel)); 
       if (is_in_history == 0 && pomdp_state_->risk_pose[i] > request_distance) {
            if (pomdp_state_->risk_pose[i] < min_dist) {
               min_dist = pomdp_state_->risk_pose[i];
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
