#pragma once
#include "rclcpp/rclcpp.hpp"

#include "despot/interface/world.h"
#include "cooperative_perception/libgeometry.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>
#include "cooperative_perception/srv/intervention.hpp"
#include "cooperative_perception/srv/state.hpp"
#include "cooperative_perception/srv/update_perception.hpp"


using namespace despot;

class CPWorld: public World, public rclcpp::Node {
private:
    int argc; char* argv[];
    // recognition result
    std::map<std::int8_t, unique_identifier_msgs::msg::UUID> id_idx_list_;

    // store previous state
    std::shared_ptr<CPState> pomdp_state_ = std::make_shared<CPState>();
    std::vector<unique_identifier_msgs::msg::UUID> req_taret_history_;
    std::vector<OBS_TYPE> obs_history_;

    // act, obs -> target index mapping
    std::shared_ptr<CPValues> cp_values_ = std::make_shared<CPValues>();


public:
    CPWorld(int argc, char* argv[]);
    ~CPWorld();
    State* Initialize();
    bool Connect(int argc, char* argv[]);
    bool Connect();
    State* GetCurrentState();
    void GetCurrentState(State &state, std::vector<double> &likelihood_list);
    bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);
    void UpdatePerception(const ACT_TYPE &action, const OBS_TYPE &obs, const std::vector<double> &risk_probs);

    ACT_TYPE MyopicAction();
    ACT_TYPE EgoisticAction(); 

private:
    rclcpp::Client<cooperative_perception::srv::Intervention>::SharedPtr intervention_client_;
    rclcpp::Client<cooperative_perception::srv::State>::SharedPtr current_state_client_;
    rclcpp::Client<cooperative_perception::srv::UpdatePerception>::SharedPtr update_perception_client_;


}; 

