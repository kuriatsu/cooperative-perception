#include "ras_world.h"

using namespace despot;

RasWorld::RasWorld() {

}

RasWorld::RasWorld(VehicleModel *vehicle_model_, OperatorModel *operator_model_, double delta_t, double obstacle_density_, std::vector<double> perception_range, std::string policy_type_) {

    operator_model = operator_model_;
    vehicle_model = vehicle_model_;
    policy_type = policy_type_;
    obstacle_density = obstacle_density_
    sim = new SumoInterface(vehicle_model, delta_t, obstacle_density_, perception_range);
}

RasWorld::~RasWorld() {

    m_log["obstacle_density"] = obstacle_density;
    m_log["policy"] = policy_type;
    m_log["delta_t"] =vehicle_model->m_delta_t ;

    time_t now = std::time(nullptr);
    struct tm* local_now = std::localtime(&now);
    std::stringstream ss;
    ss << policy_type+std::to_string(obstacle_density)+"_" 
       << local_now->tm_year + 1900
       << setw(2) << setfill('0') << local_now->tm_mon 
       << setw(2) << setfill('0') << local_now->tm_mday 
       << setw(2) << setfill('0') << local_now->tm_hour
       << setw(2) << setfill('0') << local_now->tm_min
       << setw(2) << setfill('0') << local_now->tm_sec
       << ".json";
    std::ofstream o(ss.str());
    o << std::setw(4) << m_log << std::endl;
    std::cout << "saved log data to " << ss.str() << std::endl;

    exit(1);
    std::cout << "#### close simulator ####" << std::endl;
    sim->close();
}

bool RasWorld::Connect(){
    sim->start();
    return true;
}

State* RasWorld::Initialize() {
    sim->spawnPedestrians();
    sim->spawnEgoVehicle();
    pomdp_state = new TAState();
    pomdp_state->req_time = 0;
    pomdp_state->req_target = 0;
    ta_values = new TAValues(); 
    return NULL;
}


State* RasWorld::GetCurrentState() {

    std::cout << "################GetCurrentState##################" << std::endl;
    perception_targets = sim->perception();

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

    ta_values = new TAValues(pomdp_state->risk_pose.size());

    State* out_state = static_cast<State*>(pomdp_state);
    return out_state;
}

std::vector<double> RasWorld::GetPerceptionLikelihood() {
    std::vector<double> likelihood;
    for (const auto& risk: perception_targets) {
        likelihood.emplace_back(risk.risk_prob);
    }
    return likelihood;
}

bool RasWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {

    std::cout << "execute action" << std::endl;
    TAValues::ACT ta_action = ta_values->getActionAttrib(action);
    int target_idx = ta_values->getActionTarget(action);
    
    // NO_ACTION
    if (ta_action == TAValues::NO_ACTION) {
        std::cout << "NO_ACTION" << std::endl;
        obs = operator_model->execIntervention(pomdp_state->req_time, pomdp_state->ego_recog[pomdp_state->req_target]);
        obs_history.emplace_back(obs);

        if (pomdp_state->req_time > 0) {
            sim->getRisk(id_idx_list[pomdp_state->req_target])->risk_pred = obs;
            pomdp_state->ego_recog[pomdp_state->req_target] = obs;
        }

        pomdp_state->req_time = 0;
        req_target_history.emplace_back("none");
        ta_values->printObs(obs);

    }
    
    // intervention request
    else {
        std::string req_target_id = id_idx_list[target_idx];
        std::cout << "action : REQUEST to " << target_idx << " = " << req_target_id << std::endl;
        obs = operator_model->execIntervention(pomdp_state->req_time, pomdp_state->ego_recog[pomdp_state->req_target]);
        obs_history.emplace_back(obs);

		// request to the same target or started to request
        if (req_target_history.empty() || req_target_history.back() == req_target_id || pomdp_state->req_time == 0) {
            pomdp_state->req_time += Globals::config.time_per_move;
            sim->getRisk(req_target_id)->risk_pred = TAValues::RISK;
            pomdp_state->ego_recog[target_idx] = TAValues::RISK;
        }
		// change request target 
        else {
            sim->getRisk(req_target_id)->risk_pred = TAValues::RISK;
            pomdp_state->ego_recog[target_idx] = TAValues::RISK;
            sim->getRisk(id_idx_list[pomdp_state->req_target])->risk_pred = obs;
            pomdp_state->req_time = Globals::config.time_per_move;
            pomdp_state->req_target = target_idx;
        }

        std::vector<int> red_color = {200, 0, 0};
        sim->setColor(req_target_id, red_color, "p");
        req_target_history.emplace_back(req_target_id);
        ta_values->printObs(obs);

    }
    return false;
}


void RasWorld::UpdateState(ACT_TYPE action, OBS_TYPE obs, const std::vector<double>& risk_probs) {
    // Reflect belief to risk database in sumo_interface
    for (auto itr = risk_probs.begin(), end = risk_probs.end(); itr != end; itr++) {
        int idx = std::distance(risk_probs.begin(), itr);
        std::string req_target_id = id_idx_list[idx];
        Risk* risk = sim->getRisk(req_target_id);
        risk->risk_prob = *itr;
        risk->risk_pred = pomdp_state->ego_recog[idx];
    }

    sim->controlEgoVehicle(perception_targets);
}
     
void RasWorld::Log(ACT_TYPE action, OBS_TYPE obs) {
    double time;
    Pose ego_pose;
    std::vector<double> other_ego_info;
    std::vector<Risk> log_risks;
    sim->log(time, ego_pose, other_ego_info, log_risks);

    std::string log_action = ta_values->getActionName(action);
    std::string log_obs = ta_values->getObsName(obs);
    std::string log_action_target = "NONE";
    if (log_action != "NO_ACTION") {
        log_action_target = id_idx_list[ta_values->getActionTarget(action)];
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

    m_log["log"].emplace_back(step_log);
}

bool RasWorld::isTerminate() {
    return sim->isTerminate();
}

void RasWorld::Step(int delta_t) {
    sim->step(delta_t);
}

ACT_TYPE RasWorld::MyopicAction() {

    // if intervention requested to the target and can request more
    if (0 < pomdp_state->req_time && pomdp_state->req_time < 6 && pomdp_state->risk_pose[pomdp_state->req_target] > vehicle_model->getDecelDistance(pomdp_state->ego_speed, vehicle_model->m_max_decel, 0.0)) {
        return ta_values->getAction(TAValues::REQUEST, pomdp_state->req_target);
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
        return ta_values->getAction(TAValues::REQUEST, closest_target);
    }
    else
        return ta_values->getAction(TAValues::NO_ACTION, 0);
}

ACT_TYPE RasWorld::EgoisticAction() {
    return ta_values->getAction(TAValues::NO_ACTION, 0);
}

