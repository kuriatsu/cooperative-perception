#pragma once

class OperatorModel {
public:
    int min_time;
    double acc_time_min;
    double acc_time_slope;

public:
    OperatorModel();
        min_time(3.0),
        acc_time_min(0.5),
        acc_time_slope(0.25){
        }

    OperatorModel(int _min_time, int _acc_time_min, double _acc_time_slope):
        min_time(_min_time),
        acc_time_min(_acc_time_min),
        acc_time_slope(_acc_time_slope){
        }
    ~OperatorModel();
	
	double int_acc(const int time) const;
};
