#pragma once
#include "despot/interface/world.h"
#include "libgeometry.h"

#include "rclcpp/rclcpp.hpp"

using namespace despot;

class RasWorld: public World, public rclcpp::Node {
private:
    // recognition result
    std::vector<Risk> _perception_targets;
    std::map<std::int8_t, unique_identifier_msgs::msg::UUID> _id_idx_list;

    // for myopic action
    std::vector<unique_identifier_msgs::msg::UUID> _req_target_history;
    std::vector<OBS_TYPE> _obs_history;

    // ros msg

    // 
    std::shared_ptr<CPValues> _cp_values = std::make_shared<CPValues>();
    std::shared_ptr<CPState> _pomdp_state = std::make_shared<CPState>(); // save previous state


public:
    RasWorld();
    RasWorld(VehicleModel *vehicle_model_, OperatorModel *operator_model_, double delta_t, double obstacle_density, std::vector<double> perception_range, std::string log_file_prefix_); 
    bool Connect();
    State* Initialize();
    State* GetCurrentState();
    std::vector<double> GetPerceptionLikelihood();
    bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);
    void UpdateState(ACT_TYPE action, OBS_TYPE obs, const std::vector<double>& risk_probs);
    void Log(ACT_TYPE action, OBS_TYPE obs);
    void Step(int delta_t = 0);
    bool IsTerminate();

    ACT_TYPE MyopicAction();
    ACT_TYPE EgoisticAction(); 
    ~RasWorld();

private:
    rclcpp::Client<cooperative_perception::srv::Intervention>::SharedPtr _intervention_client;
    rclcpp::Client<cooperative_perception::srv::State>::SharedPtr _current_state_client;
    rclcpp::Client<cooperative_perception::srv::UpdatePerception>::SharedPtr _update_perception_client;


}; 

