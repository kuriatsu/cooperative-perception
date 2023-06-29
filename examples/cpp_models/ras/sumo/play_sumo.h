#pragma once

#include <iostream>
#include <libsumo/libtraci.h>


using namespace libtraci;

class SumoSimulation {
public:
    int m_density = 0.1; // 1ppl per 1m
    double m_risk_thresh = 0.5;
    double m_v_min_decel = 2.0 * 9.8;
    double m_v_accel = 2.5 * 9.8;
    double m_v_max_speed = 11.2;
    double m_v_yield_speed = 2.8;
    double m_safety_margin = 5.0;
    std::vector<double> m_perception_range = {50, 150}; // x l&r, y_forward

public:
    void spawnEgoVehicle();
    void spawnPedestrians();
    void controlEgoVehicle();
    void perception();

private:
    std::unordered_map<std::string, std::vector<double, double>> m_risks;
     
};
