#pragma once

#include "despot/core/solver.h"
#include "cooperative_perception/vehicle_model.hpp"
#include "cooperative_perception/operator_model.hpp"
#include "cooperative_perception/cp_pomdp.hpp"
#include "cooperative_perception/libgeometry.hpp"

class ModelbasePlanner: public Solver
{
    public:
        VehicleModel *vehicle_model_;
        OperatorModel *operator_model_;
        CPState *cp_state_;
        CPValues *cp_values_;

    public:
        ModelbasePlanner();
        ModelbasePlanner(VehicleModel* vehicle_model, OperatorModel* operator_model, CPState* cp_state, CPValues* cp_values):
            vehicle_model_(vehicle_model),
            operator_model_(operator_model),
            cp_state_(cp_state),
            cp_values_(cp_values) {};

}

class MyopicModel: public ModelbasePlanner
{
    public:
        MyopicModel(){}
        double request_time_ = 6.0;
        ValuedAction Search();
};

class NoRequestModel: public ModelbasePlanner
{
    public:
        NoRequestModel(){}
        ValuedAction Search();
}
