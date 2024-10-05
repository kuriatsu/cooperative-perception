#include "cooperative_perception/cp_world.hpp"

CPWorld::CPWorld()
{
    cp_state_ = new CPState();
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
    return true;
}

bool CPWorld::Connect(int argc, char* argv[])
{ 
    rclcpp::init(argc, argv);
    node_ = rclcpp::Node::make_shared("CPWorldNode");

    intervention_client_ = node_->create_client<cooperative_perception::srv::Intervention>("/intervention");
    current_state_client_ = node_->create_client<cooperative_perception::srv::State>("/cp_current_state");
    update_perception_client_ = node_->create_client<cooperative_perception::srv::UpdatePerception>("/cp_updated_target");

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


State* CPWorld::GetCurrentState(std::vector<double> &likelihood_list, const double risk_thresh) 
{

    /* request to service */
    auto request = std::make_shared<cooperative_perception::srv::State::Request>();
    request->request = true;

    while (!current_state_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node_->get_logger(), "[cp_world::GetCurrentState] Interrupted while waiting for service. Exit");
            return nullptr;
        } 
        RCLCPP_INFO(node_->get_logger(), "[cp_world::GetCurrentState] service not available");
    }

    auto result = current_state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "[cp_world::GetCurrentState] failed to call service Intervention");
        return nullptr;
    }

    RCLCPP_INFO(node_->get_logger(), "[GetCurrentState] update cp_state_, id_idx_list_, likelihood_list ");
    /* start making current state*/
    // check wether last request target still exists in the perception targets
    bool is_last_req_target_exist = false;

    std::shared_ptr<cooperative_perception::srv::State::Response> buf_result = result.get();
    cp_state_->risk_pose.clear();
    cp_state_->ego_recog.clear();
    cp_state_->risk_bin.clear();
    cp_state_->risk_type.clear();
    cp_state_->ego_pose = 0;
    cp_state_->ego_speed = buf_result->ego_speed;

    id_idx_list_.clear();



    for (auto it = buf_result->object_id.begin(), end = buf_result->object_id.end(); it != end; ++it) 
    {
        int i = std::distance(buf_result->object_id.begin(), it);
        id_idx_list_[i] = *it;
        double &likelihood = buf_result->likelihood[i];
        likelihood_list.emplace_back(likelihood);

        cp_state_->ego_recog.emplace_back(likelihood>risk_thresh);
        cp_state_->risk_bin.emplace_back(likelihood>risk_thresh);
        cp_state_->risk_pose.emplace_back(buf_result->risk_pose[i]);
        cp_state_->risk_type.emplace_back(buf_result->type[i].data);

        /* is this request target (index can change at each time step) */
        if (req_target_history_.size() > 0 && req_target_history_.back().uuid == buf_result->object_id[i].uuid) {
            is_last_req_target_exist = true;
            cp_state_->req_target = i; 
        }

        std::stringstream ss;
        ss << "[GetCurrentState]" << "\n"
           << "id   :" << i << "\n" 
           << "pose :" << cp_state_->risk_pose[i] << "\n" 
           << "prob :" << likelihood << "\n";
        RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    }


    // check last request and update request time
    if (!is_last_req_target_exist) 
    {
        cp_state_->req_time = 0;
        RCLCPP_INFO(node_->get_logger(), "[GetCurrentState] new request state");
    } 
    else  {
        RCLCPP_INFO(node_->get_logger(), "[GetCurrentState] continue request");
    }

    cp_values_ = new CPValues(cp_state_->risk_pose.size());
    // state = dynamic_cast<CPState>(*cp_state_);
    std::cout << cp_state_->risk_pose.size() << std::endl;
    return cp_state_;
}


bool CPWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {
    return false;
}

bool CPWorld::CPExecuteAction(ACT_TYPE &action, OBS_TYPE& obs) {

    stringstream ss;
    ss << "[CPExecuteAction] execute action " << cp_values_->getActionName(action);
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());


    /* request to service */
    auto request = std::make_shared<cooperative_perception::srv::Intervention::Request>();
    
    /* intervention request */
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
    /* no request */
    {
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
            RCLCPP_ERROR(node_->get_logger(), "[CPExecuteAction] Interrupted while waiting for service. Exit");
            return false;
        } 
        RCLCPP_INFO(node_->get_logger(), "[CPExecuteAction] service not available");
    }

    /* send request */
    auto result = intervention_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "[CPExecuteAction] failed to call service Intervention");
        return false;
    }

    /* process request result (observation) */
    std::shared_ptr<cooperative_perception::srv::Intervention::Response> result_get = result.get();
    auto intervention_target_id = result_get->object_id.uuid;

    /* action of the obs can be different
     * because of time delay of operator intervention
     */
    bool is_intervention_target_found = false;
    for (const auto &itr : id_idx_list_) {
        std::cout 
            << static_cast<int>(itr.second.uuid[0])
            << static_cast<int>(itr.second.uuid[1])
            << static_cast<int>(itr.second.uuid[2])
            << static_cast<int>(itr.second.uuid[3])
            << static_cast<int>(itr.second.uuid[4])
            << static_cast<int>(itr.second.uuid[5])
            << static_cast<int>(itr.second.uuid[6])
            << static_cast<int>(itr.second.uuid[7])
            << static_cast<int>(itr.second.uuid[8])
            << static_cast<int>(itr.second.uuid[9])
            << static_cast<int>(itr.second.uuid[10])
            << static_cast<int>(itr.second.uuid[11])
            << static_cast<int>(itr.second.uuid[12])
            << static_cast<int>(itr.second.uuid[13])
            << static_cast<int>(itr.second.uuid[14])
            << static_cast<int>(itr.second.uuid[15])
            << std::endl;
        if (itr.second.uuid == intervention_target_id) {
            /* intervention result is the result of action at last time step */
            action = cp_values_->getAction(CPValues::REQUEST, itr.first);
            obs = result_get->result;
            is_intervention_target_found = true;
            RCLCPP_INFO(node_->get_logger(), "[CPExecuteAction] intervention target found");
            cp_values_->printObs(obs);
            std::cout << cp_values_->getActionTarget(action) << std::endl;
            break;
        }
    }

    if (!is_intervention_target_found) {
        RCLCPP_INFO(node_->get_logger(), "[CPExecuteAction] intervention target no longer exist");
        obs = CPValues::RISK;
    }


    obs_history_.emplace_back(obs);
    return false;
}


void CPWorld::UpdatePerception (const ACT_TYPE &action, const OBS_TYPE &obs, const std::vector<double> &risk_probs)
{
    if (cp_values_->getActionAttrib(action) == CPValues::NO_ACTION) {
        RCLCPP_INFO(node_->get_logger(), "[UpdatePerception] NO_ACTION");
        return;
    }
    int target_index = cp_values_->getActionTarget(action);

    /* request instance */
    auto request = std::make_shared<cooperative_perception::srv::UpdatePerception::Request> ();
    request->object_id = id_idx_list_[target_index];
    request->likelihood = risk_probs[target_index];

    /* throw request */
    while (!update_perception_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "[UpdatePerception] Interrupted while waiting for service. Exit");
            return;
        } 

        RCLCPP_INFO(node_->get_logger(), "[UpdatePerception] service not available");
    }

    /* get request */
    auto result = update_perception_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR (node_->get_logger(), "[UpdatePerception] failed to call service Intervention");
        return;
    }
}


