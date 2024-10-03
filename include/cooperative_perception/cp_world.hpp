#pragma once
#include "rclcpp/rclcpp.hpp"

#include "despot/interface/world.h"
#include "cooperative_perception/libgeometry.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>
#include "cooperative_perception/srv/intervention.hpp"
#include "cooperative_perception/srv/state.hpp"
#include "cooperative_perception/srv/update_perception.hpp"


using namespace despot;

class CPWorld: public World {
private:

    // store previous state
    CPState* cp_state_;
    std::vector<OBS_TYPE> obs_history_;

    // act, obs -> target index mapping
    CPValues* cp_values_;
    std::shared_ptr<rclcpp::Node> node_;

public:
    // recognition result
    std::map<int, unique_identifier_msgs::msg::UUID> id_idx_list_;
    std::vector<unique_identifier_msgs::msg::UUID> req_target_history_;

public:
    CPWorld ();
    ~CPWorld ();
    State* Initialize ();
    bool Connect(int argc, char* argv[]);
    bool Connect();
    void Step();
    State* GetCurrentState ();
    State* GetCurrentState (std::vector<double> &likelihood_list, const double risk_thresh);
    bool ExecuteAction (ACT_TYPE action, OBS_TYPE &obs);
    bool CPExecuteAction (ACT_TYPE &action, OBS_TYPE &obs);
    void UpdatePerception (const ACT_TYPE &action, const OBS_TYPE &obs, const std::vector<double> &risk_probs);


private:
    rclcpp::Client<cooperative_perception::srv::Intervention>::SharedPtr intervention_client_;
    rclcpp::Client<cooperative_perception::srv::State>::SharedPtr current_state_client_;
    rclcpp::Client<cooperative_perception::srv::UpdatePerception>::SharedPtr update_perception_client_;


}; 

