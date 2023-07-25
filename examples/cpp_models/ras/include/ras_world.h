#pragma once
#include "task_allocation.h"
#include "despot/interface/world.h"
#include "sumo_interface.h"
#include "operator_model.h"
#include "libgeometry.h"

using namespace despot;

class RasWorld: public World {
private:
    TAState* pomdp_state; // save previous state
    std::vector<std::string> id_idx_list;
    std::vector<std::string> perception_targets;

public:
    SumoInterface* sim;
    OperatorModel* operator_model;
    TAValues* ta_values;

public:
    RasWorld();
    RasWorld(double max_speed, double yield_speed, double max_accel, double max_decel, int safety_margin, double delta_t, double obstacle_density, std::vector<double> perception_range); 
    bool Connect();
    State* Initialize();
    State* GetCurrentState();
    State* GetCurrentState(std::vector<double>& likelihood);
    bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);
    void UpdateState(ACT_TYPE action, OBS_TYPE obs, const std::vector<double>& risk_probs);
    void Step();
    ~RasWorld();
}; 

