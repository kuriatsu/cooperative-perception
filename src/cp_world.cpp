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


State* CPWorld::GetCurrentState() {
    return static_cast<State*>(_pomdp_state);
}


void CPWorld::GetCurrentState(State &state, std::vector<double> &likelihood_list) 
{

    /* request to service */
    auto request = std::make_shared<cooperative_perception::srv::State::Request>();
    request->request = true;

    while (!_current_state_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node] Interrupted while waiting for service. Exit");
            return 0;
        } 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world_node] service not available");
    }

    auto result = _current_state_client->async_send_request(request);
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

    _id_idx_list.clear();

    for (int i=0; i<sizeof(result.get()->ids)/sizeof(unique_identifier_msgs::msg::UUID); i++) 
    {
        _id_idx_list[i] = result.get()->object_id[i];
        double &likelihood = result.get()->likelihood[i];
        likelihood_list.emplace_back(likelihood);

        _cp_state->ego_recog.emplace_back(likelihood>_risk_thresh);
        _cp_state->risk_bin.emplace_back(likelihood>_risk_thresh);

        /* is this request target (index can change at each time step) */
        if (_req_target_history.size() > 0 && _req_target_history.back() == result.get()->object_id[i]) {
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

    _cp_values = new CPState(_cp_state->risk_pose.size());

    state = static_cast<State*>(_cp_state);
}




bool CPWorld::ExecuteAction(ACT_TYPE &action, OBS_TYPE& obs) {

    std::cout << "execute action" << std::endl;

    /* request to service */
    auto request = std::make_shared<cooperative_perception::srv::Intervention::Request>();
    
    // intervention request
    if (_cp_values->getActionAttrib(action) == CPState::REQUEST) {
        int req_target_idx = _cp_values->getActionTarget(action);
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

        _req_target_history.emplace_back(req_target_id);

        request->action = CPValues::REQUEST;
        request->object_id = req_target_id;

    } else 
    {
        std::cout << "NO_ACTION" << std::endl;
        _cp_state->req_time = 0;
        _cp_state->req_target = 0;

        _req_target_history.emplace_back("none");

        request->action = CPValues::NO_ACTION;
        request->object_id = 0;
    }

    /* throw request */
    while (!_intervention_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node] Interrupted while waiting for service. Exit");
            return 0;
        } 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world_node] service not available");
    }

    /* send request */
    auto result = _intervention_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node] failed to call service Intervention");
        return;
    }

    /* process request result (observation) */
    auto intervention_target_id = result.get()->object_id;
    std::int8_t intervention_result = result.get()->result;
    for (const auto &itr : _id_idx_list) 
    {
        if (itr.second == intervention_target_id)
        {
            /* intervention result is the result of action at last time step */
            action = _cp_values->getAction(CPValues::REQUEST, itr.first);
            obs = intervention_result;
            break;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world_node] intervention target no longer exist");
    }

    obs_history.emplace_back(obs);
    cp_values->printObs(obs);
}


void CPWorld::UpdatePerception(const ACT_TYPE &action, const OBS_TYPE &obs, const std::vector<double> &risk_probs)
{
    std::int8_t target_index = _cp_values->getActionTarget(action);
    unique_identifier_msgs::msg::UUID target_id = ;

    auto request = std::make_shared<cooperative_perception::srv::UpdatePerception::Request>();
    request->object_id = _id_idx_list[target_index];
    request->likelihood = risk_probs[target_index];

    /* throw request */
    while (!_update_perception_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node::update_perception_client] Interrupted while waiting for service. Exit");
            return;
        } 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world_node::update_perception_client] service not available");
    }

    /* send request */
    auto result = _update_perception_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world_node::update_perception_client] failed to call service Intervention");
        return;
    }
}

std::vector<double> CPPOMDP::GetRiskProb(const Belief* belief) {
	const vector<State*>& particles = static_cast<const ParticleBelief*>(belief)->particles();
	
	// double status = 0;
	vector<double> probs(_cp_state->risk_pose.size(), 0.0);
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		CPState* state = static_cast<CPState*>(particle);
		for (auto itr=state->risk_bin.begin(), end=state->risk_bin.end(); itr!=end; itr++) {
			probs[distance(state->risk_bin.begin(), itr)] += *itr * particle->weight;
		}
	}
    return probs;
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

