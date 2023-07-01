#pragma once

#include <iostream>
#include <libsumo/libtraci.h>
#include <unordered_map>
#include <random>
#include <algorithm>
#include "libgeometry.h"

using namespace libtraci;

class SumoSimulation {
public:
    double m_density = 0.1; // 1ppl per 1m
    double m_risk_thresh = 0.5;
    double m_v_min_decel = 2.0 * 9.8;
    double m_v_accel = 2.5 * 9.8;
    double m_v_max_speed = 11.2;
    double m_v_yield_speed = 2.8;
    double m_safety_margin = 5.0;
    std::vector<double> m_perception_range = {50, 150}; // x l&r, y_forward
    std::string m_ego_name = "ego_vehicle";

public:
    SumoSimulation();
    std::vector<std::string> perception();
    void controlEgoVehicle(const std::vector<std::string>& targets);
    void spawnPedestrians();
    void spawnEgoVehicle();
    
private:
    std::unordered_map<std::string, Risk> m_risks;
     
};
