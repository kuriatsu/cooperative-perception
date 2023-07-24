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
        pomdp_state->ego_recog.emplace_back(risk.risk_pred);
        pomdp_state->risk_pose.emplace_back(risk.distance);
        pomdp_state->risk_bin.emplace_back(risk.risk_hidden);
    }

    NO_ACTION = pomdp_state->risk_pose.size();
    RECOG = NO_ACTION+1;

    State* out_state = static_cast<State*>(pomdp_state);
    return out_state;
}

State* RasWorld::GetCurrentState(std::vector<double>& likelihood) {

    perception_targets = sim->perception();

    id_idx_list.clear();
    likelihood.clear();

    pomdp_state->ego_pose = 0;
    pomdp_state->ego_speed = sim->getEgoSpeed();
    pomdp_state->ego_recog.clear();
    pomdp_state->risk_pose.clear();
    pomdp_state->risk_bin.clear();
    for (const auto& risk: sim->getRisk(perception_targets)) {
        id_idx_list.emplace_back(risk.id);
        likelihood.emplace_back(risk.risk_prob);
        pomdp_state->ego_recog.emplace_back(risk.risk_pred);
        pomdp_state->risk_pose.emplace_back(risk.distance);
        pomdp_state->risk_bin.emplace_back(risk.risk_hidden);
        std::cout << 
            "id :" << risk.id << "\n" <<
            "distance :" << risk.distance << "\n" <<
            "pred :" << risk.risk_pred << "\n" <<
            "prob :" << risk.risk_prob << "\n" <<
            std::endl;
    }

    NO_ACTION = pomdp_state->risk_pose.size();
    RECOG = NO_ACTION+1;

    State* out_state = static_cast<State*>(pomdp_state);
    return out_state;
}


bool RasWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {

    // intervention request
    if (REQUEST <= action && action < NO_ACTION) {
        int req_target_idx = action - REQUEST;
        std::string req_target_id = id_idx_list[req_target_idx];
        if (pomdp_state->req_target == req_target_idx) {
            pomdp_state->req_time++;
            obs = operator_model->execIntervention(pomdp_state->req_time, "REQUEST", req_target_id, sim->getRisk(req_target_id)->risk_hidden);
        }
        else {
            sim->getRisk(req_target_id)->risk_pred = true;
            pomdp_state->ego_recog[req_target_idx] = true;
            pomdp_state->req_time = 1;
            pomdp_state->req_target = req_target_idx;
            obs = operator_model->execIntervention(pomdp_state->req_time, "REQUEST", req_target_id, sim->getRisk(req_target_id)->risk_hidden);
        }
    }
    // change recog state
    else if (RECOG < action) {
        int recog_target_idx = action - RECOG;
        std::string recog_target_id = id_idx_list[recog_target_idx];

        sim->getRisk(recog_target_id)->risk_pred = (sim->getRisk(recog_target_id)->risk_pred) ? false : true;
        pomdp_state->ego_recog[recog_target_idx] = (pomdp_state->ego_recog[recog_target_idx]) ? false : true;
        pomdp_state->req_time = 1;
        pomdp_state->req_target = NONE;
        obs = operator_model->execIntervention(pomdp_state->req_time, "RECOG", recog_target_id, sim->getRisk(recog_target_id)->risk_hidden);
    }
    // NO_ACTION
    else {
        pomdp_state->req_time = 1;
        pomdp_state->req_target = NONE;
        obs = operator_model->execIntervention(pomdp_state->req_time, "NO_ACTION", "", false);
    }
    return false;
}


void RasWorld::UpdateState(ACT_TYPE action, OBS_TYPE obs, const std::vector<double>& risk_probs) {
    std::cout << "risk_prob when update " << std::string(risk_probs.begin(), risk_probs.end());
    for (auto itr = risk_probs.begin(), end = risk_probs.end(); itr != end; itr++) {
        int idx = std::distance(risk_probs.begin(), itr);
        std::string req_target_id = id_idx_list[idx];
        Risk* risk = sim->getRisk(req_target_id);
        risk->risk_prob = *itr;
        std::cout << "risk prob of : " <<  risk->risk_prob << std::endl;
        risk->risk_pred = pomdp_state->ego_recog[idx];
    }

    sim->controlEgoVehicle(perception_targets);
}
     

void RasWorld::Step() {
    sim->step();
}
