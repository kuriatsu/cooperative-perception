#pragma once
#include <iostream>
#include <despot/util/random.h>
#include "libgeometry.h"

class OperatorModel {
public:
    double min_time;
    double acc_time_min;
    double acc_time_slope;
    double max_acc;
    
public:
    OperatorModel():
        min_time(3.0),
        acc_time_min(0.5),
        acc_time_slope(0.25),
        max_acc(1.0){
        }

    OperatorModel(int _min_time, double _acc_time_min, double _acc_time_slope, double _max_acc):
        min_time(_min_time),
        acc_time_min(_acc_time_min),
        acc_time_slope(_acc_time_slope),
        max_acc(_max_acc){
        }
    ~OperatorModel();
	
	double int_acc(const int time) const;
    int execIntervention(const int time, const bool risk) const;
    int execIntervention(const int time, const bool risk, const double rand_num) const;
    
private:
    enum { NO_RISK = 0, RISK = 1, NONE = 2};
};
