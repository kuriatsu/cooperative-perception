#include "ras_world.h"

using namespace despot;

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
    pomdp_state = new TAState();
    return NULL;
}

State* RasWorld::GetCurrentState(const std::vector<std::string>& target_ids) {

    id_idx_list.clear();

    pomdp_state->ego_pose = 0;
    pomdp_state->ego_speed = sim->getEgoSpeed();
    pomdp_state->ego_recog.clear();
    pomdp_state->risk_pose.clear();
    pomdp_state->risk_bin.clear();
    for (const auto& risk: sim->getRisk(target_ids)) {
        id_idx_list.emplace_back(risk.id);
        pomdp_state->ego_recog.emplace_back(risk.p_risk);
        pomdp_state->risk_pose.emplace_back(risk.distance);
        pomdp_state->risk_bin.emplace_back(risk.risk);
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
            sim->getRisk(req_target_id)->risk = true;
            pomdp_state->ego_recog[req_target_idx] = true;
            pomdp_state->req_time = 0;
            pomdp_state->req_target = req_target_idx;
        }
    }
    else if (RECOG < action) {
        sim->getRisk(req_target_id)->risk = (sim->getRisk(req_target_id)->risk) ? false : true;
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


void RasWorld::updateState(ACT_TYPE action, OBS_TYPE obs, const std::vector<double>& risk_probs) {
    for (auto itr = risk_probs.begin(), end = risk_probs.end(); itr != end; itr++) {
        std::string req_target_id = id_idx_list[std::distance(risk_probs.begin(), itr)];
        sim->getRisk(req_target_id)->p_risk = *itr;
    }
}
     



