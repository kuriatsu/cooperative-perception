#pragma once

class OperatorModel {
public:
    int min_time;
    int acc_time_min;
    double acc_time_slope;

public:
    OperatoModel();
    OperatorModel(int _min_time, int _acc_time_min, double _acc_time_slope):
        min_time(_min_time),
        acc_time_min(_acc_time_min),
        acc_time_slope(_acc_time_slope){
        }
    ~OperatorModel();

};
