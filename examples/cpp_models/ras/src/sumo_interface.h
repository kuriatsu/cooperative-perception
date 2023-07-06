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
    double m_max_speed = 11.2;
    double m_yield_speed = 2.8;
    double m_max_accel = 0.15 * 9.8;
    double m_max_decel = -0.2 * 9.8;
    double m_safety_margin = 5.0;
    double m_delta_t = 1.0;
    double m_density = 0.1; // 1ppl per 1m
    std::vector<double> m_perception_range = {50, 150}; // x l&r, y_forward

    std::string m_ego_name = "ego_vehicle";
    double m_risk_thresh = 0.5;

public:
    SumoSimulation():
        m_max_speed(11.2),
        m_yield_speed(2.8),
        m_max_accel(0.15*9.8),
        m_max_decel(0.2*9.8),
        m_safety_margin(5),
        m_delta_t(1.0),
        m_density(0.1),
        m_perception_range({50, 150}) {
    }

    SumoSimulation(double max_speed, double yield_speed, double max_accel, double max_decel, int safety_margin, double delta_t, double density, std::vector<double> perception_range):
        m_max_speed(max_speed),
        m_yield_speed(yield_speed),
        m_max_accel(max_accel),
        m_max_decel(max_decel),
        m_safety_margin(safety_margin),
        m_delta_t(delta_t),
        m_density(density),
        m_perception_range(perception_range) {
    }

    std::vector<std::string> perception();
    void controlEgoVehicle(const std::vector<std::string>& targets);
    void spawnPedestrians();
    void spawnEgoVehicle();
    void step();
    void start();
    void close();
    
private:
    std::unordered_map<std::string, Risk> m_risks;

};
