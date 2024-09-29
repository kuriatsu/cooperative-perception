#pragma once

#include "despot/core/solver.h"
#include "despot/interface/default_policy.h"
#include "cooperative_perception/vehicle_model.hpp"
#include "cooperative_perception/operator_model.hpp"
#include "cooperative_perception/cp_pomdp.hpp"
#include "cooperative_perception/cp_world.hpp"
#include "cooperative_perception/libgeometry.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

using namespace despot;

class ModelbasePlanner: public Solver
{
    public:
        VehicleModel *vehicle_model_;
        OperatorModel *operator_model_;
        CPState *cp_state_;
        CPValues *cp_values_;

    public:
        ModelbasePlanner(const DSPOMDP* model, Belief* belief)
            : Solver(model, belief) {}
};

class MyopicModel: public ModelbasePlanner
{
    public:
        double request_time_ = 6.0;
        despot::ValuedAction Search();
        std::vector<unique_identifier_msgs::msg::UUID> req_target_history_;
        std::map<int, unique_identifier_msgs::msg::UUID> id_idx_list_;

        MyopicModel(DSPOMDP* model, Belief* belief, VehicleModel* vehicle_model, OperatorModel* operator_model, World* world)
        : ModelbasePlanner(model, belief) 
        {
            vehicle_model_ = vehicle_model;
            operator_model_ = operator_model;
            CPWorld *cp_world = static_cast<CPWorld*>(world);
            cp_state_ = static_cast<CPState*>(cp_world->GetCurrentState());
            cp_values_ = new CPValues(cp_state_->risk_pose.size());
            req_target_history_ = cp_world->req_target_history_;
            id_idx_list_ = cp_world->id_idx_list_;
        }
};

class NoRequestModel: public ModelbasePlanner
{
    public:
        NoRequestModel(const DSPOMDP* model, Belief* belief, World* world)
        : ModelbasePlanner(model, belief) 
        {
            CPWorld *cp_world = static_cast<CPWorld*>(world);
            cp_state_ = static_cast<CPState*>(world->GetCurrentState());
            cp_values_ = new CPValues(cp_state_->risk_pose.size());
        }
        despot::ValuedAction Search();
};
