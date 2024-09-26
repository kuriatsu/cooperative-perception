#pragma once
#include <iostream>
#include <despot/util/random.h>
#include "libgeometry.hpp"

class OperatorModel {
public:
    double _min_time;
    double _acc_time_min;
    double _acc_time_slope;
    
public:
    OperatorModel():
        _min_time(3.0),
        _acc_time_min(0.5),
        _acc_time_slope(0.25){
        }

    OperatorModel(int min_time, double acc_time_min, double acc_time_slope):
        _min_time(min_time),
        _acc_time_min(acc_time_min),
        _acc_time_slope(acc_time_slope){
        }
    ~OperatorModel();
	
	double InterventionAccuracy(const int time) const;
    int ExecIntervention(const int time, const int action, const std::string target, const bool risk) const;
    
private:
    enum { NO_RISK = 0, RISK = 1, NONE = 2};
    mutable std::string _last_target_id = "NONE";
};
