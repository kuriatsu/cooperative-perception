#include "ras_world.h"

using namespace despot;

~World() {
    sim.close();
}

bool RasWorld::Connect(){
    sim.start();
}

State* RasWorld::Initialize() {
    sim.spawnPedestrians();
    sim.spawnEgoVehicle();
    pomdp_state = new RasState();
    return NULL;
}

State* RasWorld::GetCurretState(const std::vector<Pose>& targets) {

    id_idx_list.clear();

    pomdp_state->ego_pose = 0;
    pomdp_state->ego_speed = sim.getEgoSpeed();
    pomdp_state->ego_recog.clear();
    pomdp_state->risk_pose.clear();
    pomdp_state->risk_bin.clear();
    for (const auto& t : targets) {
        id_idx_list.emplace_back(t.first);
        pomdp_state->ego_recog.emplace_back(t.second.p_risk);
        pomdp_state->risk_pose.emplace_back(t.second.distance);
        pomdp_state->risk_bin.emplace_back(t.second.risk);
    }

    int risk_num = m_last_pomdp_state->risk_bin.size();
    NO_ACTION=risk_num;
    RECOG = risk_num + 1; 

    return pomdp_state;
}

bool RasWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {


    if (REQUEST =< action && action < NO_ACTION) {
        int req_target_idx = action - REQUEST;
        std::string req_target_id = id_idx_list[req_target_idx];
        if (pomdp_state->req_target == req_target) {
            pomdp_state->req_time++;
        }
        else {
            sim.m_risks[req_target_id].risk = true;
            pomdp_state->ego_recog[req_target_idx] = true;
            pomdp_state->req_time = 0;
            pomdp_state->req_target = req_target_idx;
        }
    }
    else if (RECOG < action) {
        sim.m_risks[req_target_id].risk = (sim.m_risks[req_target_id].risk) ? false : true;
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


void RasWorld::updateBeliefState(ACT_TYPE action, OBS_TYPE obs, const std::vector<double>& risk_probs) {
    for (auto itr = risk_probs.begin(), end = risk_probs.end(); itr != end; itr++) {
        std::string req_target_id = id_idx_list[std::distance(risk_probs.begin(), itr)];
        sim.m_risks[req_target_id].p_risk = *itr;
    }
}
     



