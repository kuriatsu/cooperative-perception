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
    pomdp_state = new TAState();
    pomdp_state->req_time = 0;
    pomdp_state->req_target = 0;
    ta_values = new TAValues(); 
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

    ta_values = new TAValues(pomdp_state->risk_pose.size());

    State* out_state = static_cast<State*>(pomdp_state);
    return out_state;
}

State* RasWorld::GetCurrentState(std::vector<double>& likelihood) {

    std::cout << "################GetCurrentState##################" << std::endl;
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

    ta_values = new TAValues(pomdp_state->risk_pose.size());

    State* out_state = static_cast<State*>(pomdp_state);
    return out_state;
}


bool RasWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {

    int target_idx;
    TAValues::ACT ta_action = ta_values->getActionTarget(action, target_idx);
     
    
    // intervention request
    if (ta_action == TAValues::REQUEST) {
        std::string req_target_id = id_idx_list[target_idx];
        std::cout << "action : REQUEST to " << target_idx << " = " << req_target_id << std::endl;

        if (pomdp_state->req_target == target_idx) {
            pomdp_state->req_time += Globals::config.time_per_move;
        }
        else {
            sim->getRisk(req_target_id)->risk_pred = TAValues::RISK;
            pomdp_state->ego_recog[target_idx] = TAValues::RISK;
            pomdp_state->req_time = Globals::config.time_per_move;
            pomdp_state->req_target = target_idx;
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

        obs = operator_model->execIntervention(pomdp_state->req_time, ta_action, "", TAValues::NO_RISK);
        ta_values->printObs(obs);
    }
    
    // NO_ACTION
    else {
        std::cout << "NO_ACTION" << std::endl;
        pomdp_state->req_time = 0;
        pomdp_state->req_target = 0;

        obs = operator_model->execIntervention(pomdp_state->req_time, ta_action, "", false);
        ta_values->printObs(obs);
    }
    return false;
}


void RasWorld::UpdateState(ACT_TYPE action, OBS_TYPE obs, const std::vector<double>& risk_probs) {
    for (auto itr = risk_probs.begin(), end = risk_probs.end(); itr != end; itr++) {
        int idx = std::distance(risk_probs.begin(), itr);
        std::string req_target_id = id_idx_list[idx];
        Risk* risk = sim->getRisk(req_target_id);
        risk->risk_prob = *itr;
        risk->risk_pred = pomdp_state->ego_recog[idx];
    }

    sim->controlEgoVehicle(perception_targets);
}
     

void RasWorld::Step() {
    sim->step();
}
