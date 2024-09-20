#include "cooperative_perception_planner/cooperative_perception_planner.hpp"

CooperativePerceptionPlanner::CooperativePerceptionPlanner(int argc, char* argv[])
    : Node("cooperative_perception_planner_node")
{
    _sub_objects = this->create_subscription<autoware_perception_msgs::msgs::PredictedObjects>("/objects/predicted_objects", 10, std::bind(&CooperativePerceptionPlanner::ObjectsCb, this, _1));
    _sub_ego_pose = this->create_subscription<geometry_msgs::msgs::PoseStamped>("/ego_vehicle/position", 10, std::bind(&CooperativePerceptionPlanner::EgoPoseCb, this, _1));
    _sub_ego_path = this->create_subscription<autoware_planning_msgs::msgs::Paths>("/ego_vehicle/path", 10, std::bind(&CooperativePerceptionPlanner::EgoPathCb, this, _1));
    _pub_updated_objects = this->create_publisher<autoware_perception_msgs::msg::PredictedObjects>("/updated_objects", 10);
    _pub_action = this->create_publisher<std_msgs::msg::Uuid>("/intervention_target", 10);
    RunPlanning(argc, argv);
}

CooperativePerceptionPlanner::~CooperativePerceptionPlanner() {
}

int CooperativePerceptionPlanner::RunPlanning(int argc, char* argv[]) 
{
    // models
    _operator_model = new OperatorModel();
    _vehicle_model = new VehicleModel(_delta_t);

    bool search_solver;
    int num_runs = 1;
    string world_type = "simulator";
    string belief_type = "DEFAULT";
    int time_limit = -1;

    option::Option *options = InitializeParamers(argc, argv, _solver_type,
                search_solver, num_runs, world_type, belief_type, time_limit);
    if(options==NULL)
        return 0;
    clock_t main_clock_start = clock();

    _model = new CPPOMDP(_planning_horizon, _risk_thresh, _vehicle_model, _operator_model, _delta_t);

}

void CooperativePerceptionPlanner::InitializeDefaultParameters() {
    Globals::config.num_scenarios = 100;
    Globals::config.sim_len = 90;
    Globals::config.time_per_move = 2.0; 
}

void CooperativePerceptionPlanner::EgoPoseCb(geometry_msgs::msg::Pose &msg) {
}

void CooperativePerceptionPlanner::EgoPathCb(autoware_perception_msgs::msg::Path &msg) {
}

void CooperativePerceptionPlanner::ObjectsCb(autoware_perception_msgs::msg::PredictedObjectsUE &msg) {
    _belief = NULL;
    _logger = NULL;
    InitializeLogger(_logger, _options, _model, _belief, _solver, num_runs,
            main_clock_start, _world, _world_type, time_limit, _solver_type);

    DisplayParameters(_options, _model);

    logger->InitRound(world->GetCurrentState());
    round_=0; step_=0;
    RasWorld* ras_world = static_cast<RasWorld*>(world);
    CPPOMDP* cp_model = static_cast<CPPOMDP*>(model);
    logger->CheckTargetTime();
    
    for (int i=0; i<delta_t; i++) {
        ras_world->Step(0);
    }

    if (ras_world->isTerminate()) {
        return true;
    }

    TAState* start_state = static_cast<TAState*>(_current_state);
    std::vector<double> likelihood_list = ras_world->GetPerceptionLikelihood(_current_state);
    RunStep(solver, world, model, logger);
}

State* CooperativePerceptionPlanner::GetCurrentState(autoware_perception_msgs::msg::PredictedObjectsUE &objects, State* state, std::vector<double> *likelihood_list, autoware_planning_msgs::msg::Path &ego_path) {

    std::cout << "################GetCurrentState##################" << std::endl;

    id_idx_list.clear();

    // check wether last request target still exists in the perception targets
    bool is_last_req_target = false;

    pomdp_state->ego_pose = 0;
    pomdp_state->ego_speed = sim->getEgoSpeed();
    pomdp_state->ego_recog.clear();
    pomdp_state->risk_pose.clear();
    pomdp_state->risk_bin.clear();
    for (int i=0; i<perception_targets.size(); i++) {
        Risk &risk = perception_targets[i];
        id_idx_list.emplace_back(risk.id);
        pomdp_state->ego_recog.emplace_back(risk.risk_pred);
        pomdp_state->risk_pose.emplace_back(risk.distance);
        pomdp_state->risk_bin.emplace_back(risk.risk_hidden);
        
        if (req_target_history.size() > 0 && req_target_history.back() == risk.id) {
            is_last_req_target = true;
            pomdp_state->req_target = i; 
        }

        std::cout << 
            "id :" << risk.id << "\n" <<
            "distance :" << risk.distance << "\n" <<
            "pred :" << risk.risk_pred << "\n" <<
            "prob :" << risk.risk_prob << "\n" <<
            std::endl;
    }

    // std::cout << req_target_history << std::endl;
    if (!is_last_req_target) {
        pomdp_state->req_time = 0;
    }
    else {
        std::cout << "req target found again, id: " << req_target_history.back() << ", idx: " << pomdp_state->req_target << std::endl;
    }

    cp_values = new CPState(pomdp_state->risk_pose.size());

    State* out_state = static_cast<State*>(pomdp_state);

    for (const auto& risk: perception_targets) {
        likelihood_list->emplace_back(risk.risk_prob);
    }
}


bool CooperativePerceptionPlanner::RunStep(State* solver, World* world, DSPOMDP* model, Logger* logger) {

    Belief* belief = cp_model->InitialBelief(start_state, likelihood_list, belief_type);
    assert(belief != NULL);
    // solver->belief(belief);

    solver = InitializeSolver(model, belief, "DESPOT", options);

    double step_start_t = get_time_second();
    double start_t = get_time_second();

    ACT_TYPE action;
    if (policy_type == "MYOPIC")
        action = ras_world->MyopicAction();
    else if (policy_type == "EGOISTIC")
        action = ras_world->EgoisticAction();
    else
        action = solver->Search().action;

    double end_t = get_time_second();
    double search_time = end_t - start_t;

    OBS_TYPE obs;
    start_t = get_time_second();
    bool terminal = ExecuteAction(action, obs);
    end_t = get_time_second();
    double execute_time = end_t - start_t;

    return false;
}

bool CooperativePerceptionPlanner::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {

    std::cout << "execute action" << std::endl;
    CPState::ACT ta_action = cp_values->getActionAttrib(action);
    int target_idx = cp_values->getActionTarget(action);
    
    // intervention request
    if (ta_action == CPState::REQUEST) {
        std::string req_target_id = id_idx_list[target_idx];
        std::cout << "action : REQUEST to " << target_idx << " = " << req_target_id << std::endl;
        std::vector<int> red_color = {200, 0, 0};
        sim->setColor(req_target_id, red_color, "p");

        if (req_target_history.empty() || req_target_history.back() == req_target_id) {
            pomdp_state->req_time += Globals::config.time_per_move;
        }
        else {
            sim->getRisk(req_target_id)->risk_pred = CPState::RISK;
            pomdp_state->ego_recog[target_idx] = CPState::RISK;
            pomdp_state->req_time = Globals::config.time_per_move;
            pomdp_state->req_target = target_idx;
        }

        req_target_history.emplace_back(req_target_id);
        obs = operator_model->execIntervention(pomdp_state->req_time, ta_action, req_target_id, sim->getRisk(req_target_id)->risk_hidden);
        obs_history.emplace_back(obs);
        cp_values->printObs(obs);
    }
    
    // change recog state
    else if (ta_action == CPState::RECOG) {
        std::string recog_target_id = id_idx_list[target_idx];
        std::cout << "action : change RECOG of " << target_idx << " = " << recog_target_id << std::endl;


        sim->getRisk(recog_target_id)->risk_pred = (sim->getRisk(recog_target_id)->risk_pred == CPState::RISK) ? CPState::NO_RISK : CPState::RISK;
        pomdp_state->ego_recog[target_idx] = (pomdp_state->ego_recog[target_idx] == CPState::RISK) ? CPState::NO_RISK : CPState::RISK;
        pomdp_state->req_time = 0;
        pomdp_state->req_target = 0;
        req_target_history.emplace_back("none");

        obs = operator_model->execIntervention(pomdp_state->req_time, ta_action, "", CPState::NO_RISK);
        obs_history.emplace_back(obs);
        cp_values->printObs(obs);
    }
    
    // NO_ACTION
    else {
        std::cout << "NO_ACTION" << std::endl;
        pomdp_state->req_time = 0;
        pomdp_state->req_target = 0;
        req_target_history.emplace_back("none");

        obs = operator_model->execIntervention(pomdp_state->req_time, ta_action, "", false);
        obs_history.emplace_back(obs);
        cp_values->printObs(obs);
    }
    return false;
}


void CooperativePerceptionPlanner::InterventionCb() {

    start_t = get_time_second();
    solver->BeliefUpdate(action, obs);
    end_t = get_time_second();
    double update_time = end_t - start_t;
    //
    // Reflect belief to risk database in sumo_interface
    for (auto itr = risk_probs.begin(), end = risk_probs.end(); itr != end; itr++) {
        int idx = std::distance(risk_probs.begin(), itr);
        std::string req_target_id = id_idx_list[idx];
        Risk* risk = sim->getRisk(req_target_id);
        risk->risk_prob = *itr;
        risk->risk_pred = pomdp_state->ego_recog[idx];
    }

    ras_world->Log(action, obs);
}

ACT_TYPE RasWorld::MyopicAction() {

    // if intervention requested to the target and can request more
    if (0 < pomdp_state->req_time && pomdp_state->req_time < 6 && pomdp_state->risk_pose[pomdp_state->req_target] > vehicle_model->getDecelDistance(pomdp_state->ego_speed, vehicle_model->m_max_decel, 0.0)) {
        return cp_values->getAction(CPState::REQUEST, pomdp_state->req_target);
    }
    // finish request and change state 
    else if (0 < pomdp_state->req_time && pomdp_state->ego_recog[pomdp_state->req_target] != obs_history.back()) {
        return cp_values->getAction(CPState::RECOG, pomdp_state->req_target);
    }

    // find request target
    int closest_target = -1, min_dist = 100000;
    for (int i=0; i<pomdp_state->risk_pose.size(); i++) {
        int is_in_history = std::count(req_target_history.begin(), req_target_history.end(), id_idx_list[i]);
        double request_distance = vehicle_model->getDecelDistance(pomdp_state->ego_pose, vehicle_model->m_min_decel, vehicle_model->m_safety_margin) + vehicle_model->m_yield_speed * (6.0 - vehicle_model->getDecelTime(pomdp_state->ego_speed, vehicle_model->m_min_decel)); 
       if (is_in_history == 0 && pomdp_state->risk_pose[i] > request_distance) {
            if (pomdp_state->risk_pose[i] < min_dist) {
               min_dist = pomdp_state->risk_pose[i];
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

ACT_TYPE RasWorld::EgoisticAction() {
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

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CooperativePerceptionPlanner>(argc, argv));
    rclcpp::shutdown();
    return 0;
}
