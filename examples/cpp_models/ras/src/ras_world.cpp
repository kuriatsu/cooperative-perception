#include "ras_world.h"

using namespace despot;

RasWorld::RasWorld() {

}

RasWorld::RasWorld(VehicleModel *vehicle_model, OperatorModel *operator_model, double delta_t, double obstacle_density, std::vector<double> perception_range, std::string policy_type, std::map<std::string, PerceptionPerformance> perception_performance, std::map<std::string, double> obstacle_type_rate) {

    _operator_model = operator_model;
    _vehicle_model = vehicle_model;
    _policy_type = policy_type;
    _obstacle_density = obstacle_density;
    _sim = new SumoInterface(_vehicle_model, delta_t, obstacle_density, perception_range, perception_performance, obstacle_type_rate);
}

bool RasWorld::Connect(){
    _sim->start();
    return true;
}

State* RasWorld::Initialize() {
    _sim->spawnPedestrians();
    _sim->spawnEgoVehicle();
    pomdp_state = new TAState();
    pomdp_state->req_time = 0;
    pomdp_state->req_target = 0;
    ta_values = new TAValues(); 
    return NULL;
}

State* RasWorld::Initialize(const std::string log_file) {
    std::ifstream i(log_file);
    nlohmann::json log_json;
    i >> log_json;
    std::vector<Risk> obj_list;
    for (auto risk : log_json["log"][0]["risks"]) {
        Risk obj;
        obj.id = risk["id"];
        obj.risk_hidden = risk["hidden"];
        obj.risk_prob = risk["prob"];
        obj.risk_pred = risk["pred"];
        obj.pose.x = risk["x"];
        obj.pose.y = risk["y"];
        obj.pose.lane = risk["lane"];
        obj.pose.lane_position = risk["lane_position"];
        obj_list.emplace_back(obj);
    }

    _sim->spawnPedestrians(obj_list);
    _sim->spawnEgoVehicle();
    pomdp_state = new TAState();
    pomdp_state->req_time = 0;
    pomdp_state->req_target = 0;
    ta_values = new TAValues(); 
    return NULL;
}


State* RasWorld::GetCurrentState() {

    std::cout << "################GetCurrentState##################" << std::endl;
    perception_target_ids = _sim->perception();
    std::cout << perception_target_ids << std::endl;

    // check wether last request target still exists in the perception targets
    bool is_last_req_target = false;

    pomdp_state->ego_pose = 0;
    pomdp_state->ego_speed = _sim->getEgoSpeed();
    pomdp_state->ego_recog.clear();
    pomdp_state->risk_pose.clear();
    pomdp_state->risk_bin.clear();
    for (int i=0; i<perception_target_ids.size(); i++) {
        Risk *risk = _sim->getRisk(perception_target_ids[i]);
        pomdp_state->ego_recog.emplace_back(risk->risk_pred);
        pomdp_state->risk_pose.emplace_back(risk->distance);
        pomdp_state->risk_bin.emplace_back(risk->risk_hidden);
        
        if (req_target_history.size() > 0 && req_target_history.back() == risk->id) {
            is_last_req_target = true;
            pomdp_state->req_target = i; 
        }

        std::cout << 
            "id :" << risk->id << "\n" <<
            "distance :" << risk->distance << "\n" <<
            "pred :" << risk->risk_pred << "\n" <<
            "prob :" << risk->risk_prob << "\n" <<
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
    for (const auto& id: perception_target_ids) {
        likelihood.emplace_back(_sim->getRisk(id)->risk_prob);
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

        pomdp_state->req_time = 0;
        req_target_history.emplace_back("none");

        obs = _operator_model->execIntervention(pomdp_state->req_time, pomdp_state->risk_bin[pomdp_state->req_target]);
        ta_values->printObs(obs);
        obs_history.emplace_back(obs);
    }
    
    /** REQEST **/
    else {
        std::string req_target_id = perception_target_ids[target_idx];
        std::cout << "action : REQUEST to " << target_idx << " = " << req_target_id << std::endl;

        pomdp_state->req_target = target_idx;
		// request to the same target or started to request
        if (req_target_history.empty() || req_target_history.back() == req_target_id || pomdp_state->req_time == 0) {
            pomdp_state->req_time += Globals::config.time_per_move;
        }
		// change request target 
        else {
            pomdp_state->req_time = Globals::config.time_per_move;
        }
        req_target_history.emplace_back(req_target_id);

        /** get obs and update it to ego_recog **/
        obs = _operator_model->execIntervention(pomdp_state->req_time, pomdp_state->risk_bin[pomdp_state->req_target]);
        ta_values->printObs(obs);
        obs_history.emplace_back(obs);

        _sim->getRisk(req_target_id)->risk_pred = obs;
        pomdp_state->ego_recog[target_idx] = obs;
        
        /** change color of the intervention target **/
        std::vector<int> red_color = {200, 0, 0};
        _sim->setColor(req_target_id, red_color, "p");

    }
    return false;
}


void RasWorld::UpdateState(ACT_TYPE action, OBS_TYPE obs, const std::vector<double>& risk_probs) {
    for (auto itr = risk_probs.begin(), end = risk_probs.end(); itr != end; itr++) {
        /* write state change to risks info in sumo_interface */
        int idx = std::distance(risk_probs.begin(), itr);
        std::string target_id = perception_target_ids[idx];
        Risk* risk = _sim->getRisk(target_id);
        risk->risk_pred = pomdp_state->ego_recog[idx];

        /* 
         * to avoid belief weight and observation bug
         * when belief prob = 1.0 and obs = NORISK -> risk_prob become nan 
         */
        if (std::isnan(*itr)) {
            risk->risk_prob = 0.5;
        }
        else {
            risk->risk_prob = *itr;
        }
    }

    /* control ego vehicle */
    _sim->controlEgoVehicle(pomdp_state->risk_pose, pomdp_state->ego_recog);
}
     
void RasWorld::Log(ACT_TYPE action, OBS_TYPE obs) {
    double time;
    Pose ego_pose;
    std::vector<double> other_ego_info;
    std::vector<Risk> log_risks;
    _sim->log(time, ego_pose, other_ego_info, log_risks);

    std::string log_action = ta_values->getActionName(action);
    std::string log_obs = ta_values->getObsName(obs);
    std::string log_action_target = "NONE";
    if (log_action != "NO_ACTION") {
        log_action_target = perception_target_ids[ta_values->getActionTarget(action)];
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

    _log["log"].emplace_back(step_log);
}

bool RasWorld::isTerminate() {
    return _sim->isTerminate();
}

void RasWorld::Step(int delta_t) {
    _sim->step(delta_t);
}

void RasWorld::Close() {
    _sim->close();
}

void RasWorld::SaveLog(std::string filename) {

    _log["obstacle_density"] = _obstacle_density;
    _log["policy"] = _policy_type;
    _log["delta_t"] = _vehicle_model->_delta_t ;
    _log["perception_acc_ave"] = _sim->_perception_acc_ave;
    _log["perception_acc_dev"] = _sim->_perception_acc_dev;
    _log["operator_max_acc"] = _operator_model->_max_acc;
    _log["operator_slope_acc_time"] = _operator_model->_slope_acc_time;
    _log["operator_min_acc"] = _operator_model->_min_acc;
    _log["operator_min_time"] = _operator_model->_min_time;


    std::ofstream o(filename);
    o << std::setw(4) << _log << std::endl;
    std::cout << "saved log data to " << filename << std::endl;
}

ACT_TYPE RasWorld::MyopicAction() {

    // if intervention requested to the target and can request more
    int request_time = 3;
    if (0 < pomdp_state->req_time && pomdp_state->req_time < request_time && pomdp_state->risk_pose[pomdp_state->req_target] > _vehicle_model->getDecelDistance(pomdp_state->ego_speed, _vehicle_model->_max_decel, 0.0)) {
        return ta_values->getAction(TAValues::REQUEST, pomdp_state->req_target);
    }

    // find request target
    int closest_target = -1, min_dist = 100000;
    for (int i=0; i<pomdp_state->risk_pose.size(); i++) {
        int is_in_history = std::count(req_target_history.begin(), req_target_history.end(), perception_target_ids[i]);
        double request_distance = _vehicle_model->getDecelDistance(pomdp_state->ego_pose, _vehicle_model->_min_decel, _vehicle_model->_safety_margin) + _vehicle_model->_yield_speed * (request_time - _vehicle_model->getDecelTime(pomdp_state->ego_speed, _vehicle_model->_min_decel)); 
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
