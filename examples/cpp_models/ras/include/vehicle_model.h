#pragma once
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

class VehicleModel {

public:
    double _max_speed   = 11.2;
    double _yield_speed = 2.8;
    double _max_accel   = 0.15*9.8;
    double _max_decel   = 0.3*9.8;
    double _min_decel   = 0.2*9.8;
    int _safety_margin  = 5;
    double _delta_t     = 1.0;

public:
    VehicleModel(); 

    VehicleModes(double max_speed, double yield_speed, double max_accel, double max_decel, double min_decel, double safety_margin, double delta_t); 
    double getDecelDistance(const double speed, const double acc, const double safety_margin) const;
    double getDecelTime(const double speed, const double acc) const;

    double getAccel(const double speed, const int pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) const;

    double clipSpeed(const double acc, const double v0) const;

    void getTransition(double& speed, int& pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) const; 
};
