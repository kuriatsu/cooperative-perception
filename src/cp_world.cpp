#include "cooperative_perception/cp_world.hpp"

CPWorld::CPWorld()
{
}

CPWorld::~CPWorld() {
    rclcpp::shutdown();
}


State* CPWorld::Initialize() 
{
    return nullptr;
}


bool CPWorld::Connect()
{ 
    int argc;
    char** argv;
    rclcpp::init(argc, argv);
    node_ = rclcpp::Node::make_shared("CPWorldNode");

    intervention_client_ = node_->create_client<cooperative_perception::srv::Intervention>("intervention");
    current_state_client_ = node_->create_client<cooperative_perception::srv::State>("cp_current_state");
    update_perception_client_ = node_->create_client<cooperative_perception::srv::UpdatePerception>("cp_updated_target");

    rclcpp::spin_some(node_);
    return true;
}


void CPWorld::Step() 
{
    rclcpp::spin_some(node_);
}

State* CPWorld::GetCurrentState() {
    return static_cast<State*>(cp_state_);
}


void CPWorld::GetCurrentState(State* state, std::vector<double> &likelihood_list, const double risk_thresh) 
{

    /* request to service */
    auto request = std::make_shared<cooperative_perception::srv::State::Request>();
    request->request = true;

    while (!current_state_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world::GetCurrentState] Interrupted while waiting for service. Exit");
            return;
        } 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world::GetCurrentState] service not available");
    }

    auto result = current_state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world::GetCurrentState] failed to call service Intervention");
        return;
    }

    /* start making current state*/
    // check wether last request target still exists in the perception targets
    bool is_last_req_target_exist = false;

    cp_state_->risk_pose.clear();
    cp_state_->ego_recog.clear();
    cp_state_->risk_bin.clear();
    cp_state_->ego_pose = 0;
    cp_state_->ego_speed = result.get()->ego_speed;
    cp_state_->risk_pose = result.get()->risk_pose;

    id_idx_list_.clear();

    for (int i=0; i<int(sizeof(result.get()->object_id)/sizeof(unique_identifier_msgs::msg::UUID)); i++) 
    {
        id_idx_list_[i] = result.get()->object_id[i];
        double &likelihood = result.get()->likelihood[i];
        likelihood_list.emplace_back(likelihood);

        cp_state_->ego_recog.emplace_back(likelihood>risk_thresh);
        cp_state_->risk_bin.emplace_back(likelihood>risk_thresh);

        /* is this request target (index can change at each time step) */
        if (req_target_history_.size() > 0 && req_target_history_.back().uuid == result.get()->object_id[i].uuid) {
            is_last_req_target_exist = true;
            cp_state_->req_target = i; 
        }


        std::cout << 
            "id :" << i << "\n" <<
            "distance :" << cp_state_->risk_pose[i] << "\n" <<
            "prob :" << likelihood << "\n" <<
            std::endl;
    }


    // check last request and update request time
    if (!is_last_req_target_exist) 
    {
        cp_state_->req_time = 0;
    } 
    // else {
    //     std::cout << "req target found again, id: " << req_target_history_.back() << ", idx: " << cp_state_->req_target << std::endl;
    // }

    cp_values_ = new CPValues(cp_state_->risk_pose.size());

    state = static_cast<State*>(cp_state_);
}


bool CPWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {
    return false;
}

bool CPWorld::CPExecuteAction(ACT_TYPE &action, OBS_TYPE& obs) {

    std::cout << "execute action" << std::endl;

    /* request to service */
    auto request = std::make_shared<cooperative_perception::srv::Intervention::Request>();
    
    // intervention request
    if (cp_values_->getActionAttrib(action) == CPValues::REQUEST) {
        int req_target_idx = cp_values_->getActionTarget(action);
        unique_identifier_msgs::msg::UUID req_target_id = id_idx_list_[req_target_idx];

        /* check intervention request */
        /* keep request to the same target */
        if (req_target_history_.empty() || req_target_history_.back().uuid == req_target_id.uuid) {
            cp_state_->req_time += Globals::config.time_per_move;
        }
        else {
            cp_state_->ego_recog[req_target_idx] = CPValues::RISK;
            cp_state_->req_time = Globals::config.time_per_move;
            cp_state_->req_target = req_target_idx;
        }

        req_target_history_.emplace_back(req_target_id);

        request->action = CPValues::REQUEST;
        request->object_id = req_target_id;

    } else 
    {
        std::cout << "NO_ACTION" << std::endl;
        cp_state_->req_time = 0;
        cp_state_->req_target = 0;

        unique_identifier_msgs::msg::UUID uuid;
        uuid.uuid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        req_target_history_.emplace_back(uuid);

        request->action = CPValues::NO_ACTION;
        request->object_id = uuid;
    }

    /* throw request */
    while (!intervention_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world::CPExecuteAction] Interrupted while waiting for service. Exit");
            return false;
        } 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world::CPExecuteAction] service not available");
    }

    /* send request */
    auto result = intervention_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world::CPExecuteAction] failed to call service Intervention");
        return false;
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world::CPExecuteAction] intervention target no longer exist");
    }

    obs_history_.emplace_back(obs);
    cp_values_->printObs(obs);
    return false;
}


void CPWorld::UpdatePerception (const ACT_TYPE &action, const OBS_TYPE &obs, const std::vector<double> &risk_probs)
{
    std::int8_t target_index = cp_values_->getActionTarget(action);

    /* request instance */
    auto request = std::make_shared<cooperative_perception::srv::UpdatePerception::Request> ();
    request->object_id = id_idx_list_[target_index];
    request->likelihood = risk_probs[target_index];

    /* throw request */
    while (!update_perception_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[cp_world::update_perception_client] Interrupted while waiting for service. Exit");
            return;
        } 

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[cp_world::update_perception_client] service not available");
    }

    /* get request */
    auto result = update_perception_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR (rclcpp::get_logger("rclcpp"), "[cp_world::update_perception_client] failed to call service Intervention");
        return;
    }
}


