#include "ras_world.h"

using namespace despot;

RasWorld::RasWorld() {

}

RasWorld::RasWorld(VehicleModel *vehicle_model, double delta_t, double obstacle_density, std::vector<double> perception_range) {

    sim = new SumoInterface(vehicle_model, delta_t, obstacle_density, perception_range);
}

RasWorld::~RasWorld() {
    time_t now = std::time(nullptr);
    struct tm* local_now = std::localtime(&now);
    std::stringstream ss;
    ss << "log"
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
        
        if (last_req_target_id == risk.id) {
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

    if (!is_last_req_target) {
        pomdp_state->req_time = 0;
    }
    else {
        std::cout << "req target found again, id: " << last_req_target_id << ", idx: " << pomdp_state->req_target << std::endl;
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
    
    // intervention request
    if (ta_action == TAValues::REQUEST) {
        std::string req_target_id = id_idx_list[target_idx];
        std::cout << "action : REQUEST to " << target_idx << " = " << req_target_id << std::endl;
        std::vector<int> red_color = {200, 0, 0};
        sim->setColor(req_target_id, red_color, "p");

        if (last_req_target_id == req_target_id) {
            pomdp_state->req_time += Globals::config.time_per_move;
        }
        else {
            sim->getRisk(req_target_id)->risk_pred = TAValues::RISK;
            pomdp_state->ego_recog[target_idx] = TAValues::RISK;
            pomdp_state->req_time = Globals::config.time_per_move;
            pomdp_state->req_target = target_idx;
            last_req_target_id = req_target_id;
        }

        obs = operator_model->execIntervention(pomdp_state->req_time, ta_action, req_target_id, sim->getRisk(req_target_id)->risk_hidden);
        ta_values->printObs(obs);
    }
    
    // change recog state
    else if (ta_action == TAValues::RECOG) {
        std::string recog_target_id = id_idx_list[target_idx];
        std::cout << "action : change RECOG of " << target_idx << " = " << recog_target_id << std::endl;


        sim->getRisk(recog_target_id)->risk_pred = (sim->getRisk(recog_target_id)->risk_pred == TAValues::RISK) ? TAValues::NO_RISK : TAValues::RISK;
        pomdp_state->ego_recog[target_idx] = (pomdp_state->ego_recog[target_idx] == TAValues::RISK) ? TAValues::NO_RISK : TAValues::RISK;
        pomdp_state->req_time = 0;
        pomdp_state->req_target = 0;
        last_req_target_id = "none";

        obs = operator_model->execIntervention(pomdp_state->req_time, ta_action, "", TAValues::NO_RISK);
        ta_values->printObs(obs);
    }
    
    // NO_ACTION
    else {
        std::cout << "NO_ACTION" << std::endl;
        pomdp_state->req_time = 0;
        pomdp_state->req_target = 0;
        last_req_target_id = "none";

        obs = operator_model->execIntervention(pomdp_state->req_time, ta_action, "", false);
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

    TAValues::ACT log_action = ta_values->getActionAttrib(action);
    std::string log_action_target = "NONE";
    if (log_action != TAValues::NO_ACTION) {
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
        {"action_target", log_action_target}
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

bool RasWorld::isTerminate() {
    return sim->isTerminate();
}

void RasWorld::Step(int delta_t) {
    sim->step(delta_t);
}

ACT_TYPE MyopicAction() {

    // if intervention requested to the target and can request more
    if (0 < pomdp_state->req_time && pomdp_state < 6 && pomdp_state->risk_pose[pomdp_state.req_target] > vehicle_model->getDecelDistance(pomdp_state->ego_speed, vehicle_model->m_max_decel, 0.0)) {
        return ta_values->getAction(TAValues::REQUEST, pomdp_state->req_target);
    }
    // finish request and change state 
    else if (0 < pomdp_state->req_time && pomdp_state->ego_recog[pomdp_state->req_target] != last_obs) {
        return ta_values->getAction(TAValues::RECOG, pomdp_satate->req_target);
    }

    // find request target
    int closest_target = -1, min_dist = 100000;
    for (int i=0; i<pomdp_state->risk_pose.size(); i++) {
        bool is_in_history = std::count(req_target_history.begin(), req_target_history.end(), id_idx_list[i]);
        double request_distance = vehicle_model->getDecelDistance(pomdp_state->ego_pose, vehicle_model->m_min_decel, vehicle_model->m_safety_margin) + vehicle_model->m_yield_speed * (6.0 - vehicle_model->getDecelTime(pomdp_state->ego_speed, vehicle_model->m_min_decel); 
       if (!is_in_history && pomdp_state->risk_pose[pomdp_state.req_target] > request_distance) {
            if (pomdp_state->risk_pose[i] < min_dist) {
               min_dist = pomdp_state->risk_pose[i];
               closest_target = i;
            }
        }
    }

    if (closest_target != -1)
        return ta_values->getAction(TAValues::REQUEST, closest_target);
    else
        return ta_values->getAction(TAValues::NO_ACTION, 0);
}

ACT_TYPE EgoisticAction() {
    return ta_values->getAction(TAValues::NO_ACTION, 0);
}

