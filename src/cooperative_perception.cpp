#include "cooperative_perception_planner/cooperative_perception.hpp"

CooperativePerceptionPlanner::CooperativePerceptionPlanner(int argc, char* argv[])
    : Node("cooperative_perception_node")
{
    _sub_objects = this->create_subscription<autoware_perception_msgs::msgs::PredictedObjects>("/objects/predicted_objects", 10, std::bind(&CooperativePerceptionPlanner::ObjectsCb, this, _1));
    _sub_ego_pose = this->create_subscription<geometry_msgs::msgs::PoseWithCovarianceStampled>("/localization/pose_with_covariance", 10, std::bind(&CooperativePerceptionPlanner::EgoPoseCb, this, _1));
    _sub_ego_speed = this->create_subscription<geometry_msgs::msgs::TwistWithCovarianceStamped>("/localization/twist", 10, std::bind(&CooperativePerceptionPlanner::EgoSpeedCb, this, _1));
    _sub_ego_traj = this->create_subscription<autoware_planning_msgs::msgs::Trajectory>("/planning/scenario_planning/trajectory", 10, std::bind(&CooperativePerceptionPlanner::EgoTrajCb, this, _1));
    _sub_intervention = this->create_subscription<cooperative_perception::msgs::InterventionTarget>("/intervention_result", 10, std::bind(&CooperativePerceptionPlanner::InterventionCb, this, _1));
    _pub_updated_objects = this->create_publisher<cooperative_perception::msg::Intervention>("/updated_objects", 10);
    _pub_action = this->create_publisher<cooperative_perception::msg::Intervention>("/intervention_target", 10);
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

    _world = NULL;
    _belief = NULL;
    _logger = NULL;
    InitializeLogger(_logger, _options, _model, _belief, _solver, num_runs,
            main_clock_start, _world, _world_type, time_limit, _solver_type);

    DisplayParameters(_options, _model);

    logger->InitRound(world->GetCurrentState());
    round_=0; step_=0;

}

void CooperativePerceptionPlanner::InitializeDefaultParameters() {
    Globals::config.num_scenarios = 100;
    Globals::config.sim_len = 90;
    Globals::config.time_per_move = 2.0; 
}

void CooperativePerceptionPlanner::EgoPoseCb(geometry_msgs::msg::PoseWithCovarianceStamped &msg) 
{
    _ego_pose = msg.pose.pose;
}

void CooperativePerceptionPlanner::EgoSpeedCb(geometry_msgs::msg::TwistWithCovarianceStamped &msg) 
{
    _ego_speed = msg.twist.twist;
}

void CooperativePerceptionPlanner::EgoTrajCb(autoware_perception_msgs::msg::Trajectory &msg) {
    _ego_traj = msg;
}

void CooperativePerceptionPlanner::InterventionCb(cooperative_perception::msg::InterventionTarget &msg) {
    // update belief
    start_t = get_time_second();
    obs = msg.intervention;
    obs_history.emplace_back(obs);
    cp_values->printObs(obs);
    solver->BeliefUpdate(action, obs);
    end_t = get_time_second();
    double update_time = end_t - start_t;

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

void CooperativePerceptionPlanner::ObjectsCb(autoware_perception_msgs::msg::PredictedObjects &msg) {
    CPPOMDP* cp_model = static_cast<CPPOMDP*>(_model);
    logger->CheckTargetTime();
    
    std::shared_ptr<CPState> start_state = std::make_shared<CPState>();
    std::vector<double> likelihood_list = ras_world->GetPerceptionLikelihood(_current_state);
    GetCurrentState(msg, start_state, likelihood_list)
    RunStep(solver, world, model, logger);
}

bool CooperativePerceptionPlanner::RunStep(State* solver, World* world, DSPOMDP* model, Logger* logger) {

    double start_t = get_time_second(), end_t;
    Belief* _belief = cp_model->InitialBelief(start_state, likelihood_list, belief_type);
    assert(_belief != NULL);
    // solver->belief(belief);

    solver = InitializeSolver(model, _belief, "DESPOT", options);


    ACT_TYPE action;
    if (policy_type == "MYOPIC")
        action = MyopicAction();
    else if (policy_type == "EGOISTIC")
        action = EgoisticAction();
    else
        action = solver->Search().action;

    end_t = get_time_second();
    double search_time = end_t - start_t;

    start_t = get_time_second();
    OBS_TYPE obs;
    bool terminal = ExecuteAction(action, obs);
    end_t = get_time_second();
    double execute_time = end_t - start_t;

    return false;
}

State* CooperativePerceptionPlanner::GetCurrentState() {
    return static_cast<State*>(_pomdp_state);
}


void CooperativePerceptionPlanner::GetCurrentState(autoware_perception_msgs::msg::PredictedObjects &objects, State* state, std::vector<double> *likelihood_list, autoware_planning_msgs::msg::Path &ego_traj) 
{

    std::cout << "################GetCurrentState##################" << std::endl;

    id_idx_list.clear();
    int target_index = 0;

    // check wether last request target still exists in the perception targets
    bool is_last_req_target = false;

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
        GetCollisionPointAndRisk(ego_traj, object.kinematics.predicted_paths, collision_prob, collision_point);

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
            is_last_req_target = true;
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
    if (!is_last_req_target) 
    {
        _pomdp_state->req_time = 0;
    } else 
    {
        std::cout << "req target found again, id: " << req_target_history.back() << ", idx: " << _pomdp_state->req_target << std::endl;
    }

    cp_values = new CPState(_pomdp_state->risk_pose.size());

    state = static_cast<State*>(_pomdp_state);
}


void CooperativePerceptionPlanner::GetCollisionPointAndRisk(const autoware_auto_planning_msgs::msg::Trajectory &ego_traj, const autoware_auto_perception_msgs::msg::PredictedPath *obj_paths, float &collision_prob, float &collision_point) 
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
                    return;
                }
            }
        }
    }
}


bool CooperativePerceptionPlanner::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {

    std::cout << "execute action" << std::endl;
    CPState::ACT ta_action = cp_values->getActionAttrib(action);
    int target_idx = cp_values->getActionTarget(action);
    
    // intervention request
    if (ta_action == CPState::REQUEST) {
        std::string req_target_id = id_idx_list[target_idx];
        std::cout << "action : REQUEST to " << target_idx << " = " << req_target_id << std::endl;

        if (req_target_history.empty() || req_target_history.back() == req_target_id) {
            pomdp_state->req_time += Globals::config.time_per_move;
        }
        else {
            pomdp_state->ego_recog[target_idx] = CPState::RISK;
            pomdp_state->req_time = Globals::config.time_per_move;
            pomdp_state->req_target = target_idx;
        }

        req_target_history.emplace_back(req_target_id);
    }
    
    
    // NO_ACTION
    else {
        std::cout << "NO_ACTION" << std::endl;
        pomdp_state->req_time = 0;
        pomdp_state->req_target = 0;
        req_target_history.emplace_back("none");

    }
    return false;
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
