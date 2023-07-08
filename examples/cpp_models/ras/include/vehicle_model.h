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
    int m_safety_margin;
    double m_delta_t;

public:
    VehicleModel() :
        m_max_speed(11.2),
        m_yield_speed(2.8),
        m_max_accel(0.15*9.8),
        m_max_decel(-0.2*9.8),
        m_safety_margin(5),
        m_delta_t(1.0) {
        }

    VehicleModel(double max_speed, double yield_speed, double max_accel, double max_decel, double safety_margin, double delta_t) :
        m_max_speed(max_speed),
        m_yield_speed(yield_speed),
        m_max_accel(max_accel),
        m_max_decel(max_decel),
        m_safety_margin(safety_margin),
        m_delta_t(delta_t) {
        }

    double getAccel(const double speed, const std::vector<bool>& recog_list, const std::vector<int>& target_poses);

    void getTransition(double& speed, int& pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses); 
};
