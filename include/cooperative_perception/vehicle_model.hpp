#pragma once
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

class VehicleModel {
public:
    double _max_speed;
    double _yield_speed;
    double _max_accel;
    double _max_decel;
    double _min_decel;
    int _safety_margin;
    double _delta_t;

public:
    VehicleModel(); 
    VehicleModel(const double delta_t);
    VehicleModel(double max_speed, double yield_speed, double max_accel, double max_decel, double min_decel, double safety_margin, double delta_t); 
    double GetDecelDistance(const double speed, const double acc, const double safety_margin) const;
    double GetDecelTime(const double speed, const double acc) const;

    double GetAccel(const double speed, const int pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) const;

    double ClipSpeed(const double acc, const double v0) const;

    void GetTransition(double& speed, int& pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) const; 
};
