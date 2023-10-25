#pragma once
#include <iostream>
#include <despot/util/random.h>
#include "libgeometry.h"

class OperatorModel {
public:
    double _min_time;
    double _min_acc;
    double _slope_acc_time;
    double _max_acc;
    
public:
    OperatorModel():
        _min_time(3.0),
        _min_acc(0.5),
        _slope_acc_time(0.25),
        _max_acc(1.0){
        }

    OperatorModel(int min_time, double min_acc, double slope_acc_time, double max_acc):
        _min_time(min_time),
        _min_acc(min_acc),
        _slope_acc_time(slope_acc_time),
        _max_acc(max_acc){
        }
    ~OperatorModel();
	
	double intAcc(const int time) const;
    int execIntervention(const int time, const bool risk) const;
    int execIntervention(const int time, const bool risk, const double rand_num) const;
    
private:
    enum { NO_RISK = 0, RISK = 1, NONE = 2};
};
