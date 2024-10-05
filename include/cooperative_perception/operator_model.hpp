#pragma once
#include <iostream>
#include <despot/util/random.h>
#include "libgeometry.hpp"

class OperatorModel {
public:
    /* min_time, min_acc, slope, max_acc, ads_mean, ads_dev */
    std::map<std::string, PerceptionPerformance> performance_{
        {"easy", {1.0, 0.9, 0.025, 0.95, 0.9, 0.1}},
        {"hard", {1.0, 0.65, 0.075, 0.8, 0.6, 0.1}},
        {"easy_plus", {1.0, 0.9, 0.01, 0.95, 0.9, 0.1}},
        {"hard_plus", {1.0, 0.65, 0.03, 0.8, 0.6, 0.1}},
        {"future", {1.0, 0.65, 0.03, 0.8, 0.9, 0.1}},
    };

public:
    OperatorModel();
    OperatorModel(const std::map<std::string, PerceptionPerformance> &perception_performance);
    ~OperatorModel();
	
    double InterventionAccuracy(const int time, const std::string type) const;
    int ExecIntervention(const int time, const bool risk, const std::string type) const;
    int ExecIntervention(const int time, const bool risk, const double rand_num, const std::string type) const;
    
};
