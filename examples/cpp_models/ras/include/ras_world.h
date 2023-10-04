#pragma once
#include "task_allocation.h"
#include "despot/interface/world.h"
#include "sumo_interface.h"
#include "operator_model.h"
#include "libgeometry.h"
#include "vehicle_model.h"

#include "nlohmann/json.hpp"

using namespace despot;

class RasWorld: public World {
private:
    TAState* pomdp_state; // save previous state
    std::vector<std::string> id_idx_list;
    std::vector<Risk> perception_targets;

    // log data
    nlohmann::json m_log;
    std::string policy_type = "DESPOT";
    double obstacle_density;

    // for myopic action
    std::vector<std::string> req_target_history;
    std::vector<OBS_TYPE> obs_history;

public:
    SumoInterface* sim;
    OperatorModel* operator_model;
    VehicleModel* vehicle_model;
    TAValues* ta_values;

public:
    RasWorld();
    RasWorld(VehicleModel *vehicle_model_, OperatorModel *operator_model_, double delta_t, double obstacle_density_, std::vector<double> perception_range, std::string policy_type_); 
    bool Connect();
    State* Initialize();
    State* GetCurrentState();
    std::vector<double> GetPerceptionLikelihood();
    bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);
    void UpdateState(ACT_TYPE action, OBS_TYPE obs, const std::vector<double>& risk_probs);
    void Log(ACT_TYPE action, OBS_TYPE obs);
    void Step(int delta_t = 0);
    bool isTerminate();

    ACT_TYPE MyopicAction();
    ACT_TYPE EgoisticAction(); 
    ~RasWorld();
}; 

