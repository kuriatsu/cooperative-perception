#include "ras_world.h"

using namespace despot;

RasWorld::RasWorld() {

}

RasWorld::RasWorld(VehicleModel *vehicle_model, double delta_t, double obstacle_density, std::vector<double> perception_range) {

    sim = new SumoInterface(vehicle_model, delta_t, obstacle_density, perception_range);
}

RasWorld::~RasWorld() {
    sim->close();
    std::ofstream o("log.json");
    o << std::setw(4) << m_log << std::endl;
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
    for (const auto& risk: perception_targets) {
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
    for (const auto& risk: perception_targets) {
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

    TAValues::ACT ta_action = ta_values->getActionAttrib(action);
    int target_idx = ta_values->getActionTarget(action);
     
    
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
    double time;
    std::vector<double> vehicle_info;
    std::vector<Risk> passed_risks;
    sim->log(time, vehicle_info, passed_risks);
    nlohmann::json step_log = {
        {"time", time},
        {"pose", vehicle_info[0]},
        {"speed", vehicle_info[2]}, 
        {"accel", vehicle_info[3]},
        {"fuel_consumption", vehicle_info[4]},
        {"passed_risks", {}}
    };

    for (const auto& risk : passed_risks) {
        nlohmann::json buf = {
            {"id", risk.id},
            {"pose", risk.pose.x},
            {"prob", risk.risk_prob},
            {"pred", risk.risk_pred},
            {"hidden", risk.risk_hidden}
        };
        step_log["risks"].emplace_back(buf);
    }

    m_log.emplace_back(step_log);
    

    for (auto itr = risk_probs.begin(), end = risk_probs.end(); itr != end; itr++) {
        int idx = std::distance(risk_probs.begin(), itr);
        std::string req_target_id = id_idx_list[idx];
        Risk* risk = sim->getRisk(req_target_id);
        risk->risk_prob = *itr;
        risk->risk_pred = pomdp_state->ego_recog[idx];
    }

    sim->controlEgoVehicle(perception_targets);
}
     
bool RasWorld::isTerminate() {
    return sim->isTerminate();
}

void RasWorld::Step(int delta_t) {
    sim->step(delta_t);
}

