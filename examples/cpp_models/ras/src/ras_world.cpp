#include "ras_world.h"

using namespace despot;

RasWorld::RasWorld() {

}

RasWorld::RasWorld(double max_speed, double yield_speed, double max_accel, double max_decel, int safety_margin, double delta_t, double obstacle_density, std::vector<double> perception_range) {

    sim = new SumoInterface(max_speed, yield_speed, max_accel, max_decel, safety_margin, delta_t, obstacle_density, perception_range);
}

RasWorld::~RasWorld() {
    sim->close();
}

bool RasWorld::Connect(){
    sim->start();
    return true;
}

State* RasWorld::Initialize() {
    sim->spawnPedestrians();
    sim->spawnEgoVehicle();
    NO_ACTION = sim->getRisk(perception_targets).size(); 
    pomdp_state = new TAState();
    pomdp_state->req_time = 0;
    pomdp_state->req_target = NO_ACTION;
    return NULL;
}

State* RasWorld::GetCurrentState() {

    perception_targets = sim->perception();

    id_idx_list.clear();

    pomdp_state->ego_pose = 0;
    pomdp_state->ego_speed = sim->getEgoSpeed();
    pomdp_state->ego_recog.clear();
    pomdp_state->risk_pose.clear();
    pomdp_state->risk_bin.clear();
    for (const auto& risk: sim->getRisk(perception_targets)) {
        id_idx_list.emplace_back(risk.id);
        pomdp_state->ego_recog.emplace_back(risk.p_risk);
        pomdp_state->risk_pose.emplace_back(risk.distance);
        pomdp_state->risk_bin.emplace_back(risk.recog_risk);
    }

    NO_ACTION = pomdp_state->risk_pose.size();
    RECOG = NO_ACTION+1;

    State* out_state = static_cast<State*>(pomdp_state);
    return out_state;
}

bool RasWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {

    int req_target_idx = action - REQUEST;
    std::string req_target_id = id_idx_list[req_target_idx];
    if (REQUEST <= action && action < NO_ACTION) {
        if (pomdp_state->req_target == req_target_idx) {
            pomdp_state->req_time++;
        }
        else {
            sim->getRisk(req_target_id)->risk_pred = true;
            pomdp_state->ego_recog[req_target_idx] = true;
            pomdp_state->req_time = 0;
            pomdp_state->req_target = req_target_idx;
        }
    }
    else if (RECOG < action) {
        sim->getRisk(req_target_id)->risk_pred = (sim->getRisk(req_target_id)->recog_risk) ? false : true;
        pomdp_state->ego_recog[req_target_idx] = (pomdp_state->ego_recog[req_target_idx]) ? false : true;
        pomdp_state->req_time = 0;
        pomdp_state->req_target = NONE;
    }
    else {
        pomdp_state->req_time = 0;
        pomdp_state->req_target = NONE;
    }
    return false;
}


void RasWorld::UpdateState(ACT_TYPE action, OBS_TYPE obs, const std::vector<double>& risk_probs) {
    for (auto itr = risk_probs.begin(), end = risk_probs.end(); itr != end; itr++) {
        std::string req_target_id = id_idx_list[std::distance(risk_probs.begin(), itr)];
        sim->getRisk(req_target_id)->risk_pred = *itr;
    }

    sim->controlEgoVehicle(perception_targets);
}
     

void RasWorld::Step() {
    sim->step();
}
