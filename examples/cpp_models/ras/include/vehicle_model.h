#pragma once
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

class VehicleModel {
public:
    double m_max_speed;
    double m_yield_speed;
    double m_max_accel;
    double m_max_decel;
    double m_min_decel;
    int m_safety_margin;
    double m_delta_t;

public:
    VehicleModel(); 

    VehicleModel(double max_speed, double yield_speed, double max_accel, double max_decel, double min_decel, double safety_margin, double delta_t); 

    double getAccel(const double speed, const int pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) const;

    double clipSpeed(const double acc, const double v0) const;

    void getTransition(double& speed, int& pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) const; 
};
