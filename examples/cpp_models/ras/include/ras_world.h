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
    std::vector<std::string> perception_target_ids;

    // log data
    nlohmann::json _log;
    std::string _policy_type = "DESPOT";
    double _obstacle_density;

    // for myopic action
    std::vector<std::string> req_target_history;
    std::vector<OBS_TYPE> obs_history;

public:
    SumoInterface* _sim;
    OperatorModel* _operator_model;
    VehicleModel* _vehicle_model;
    TAValues* ta_values;

public:
    RasWorld();
    RasWorld(VehicleModel *vehicle_model, OperatorModel *operator_model, const double delta_t, const double obstacle_density, const std::vector<double> perception_range, const std::string policy_type, const std::map<std::string, PerceptionPerformance> &perception_performance, const std::map<std::string, double> &obstacle_type_rate);
    bool Connect();
    State* Initialize();
    State* Initialize(const std::string log_file);
    State* GetCurrentState();
    std::vector<double> GetPerceptionLikelihood();
    bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);
    void UpdateState(ACT_TYPE action, OBS_TYPE obs, const std::vector<double>& risk_probs);
    void Log(ACT_TYPE action, OBS_TYPE obs);
    void SaveLog(std::string filename);
    void Step(int delta_t = 0);
    void Close();
    bool isTerminate();

    ACT_TYPE MyopicAction();
    ACT_TYPE EgoisticAction(); 
}; 

