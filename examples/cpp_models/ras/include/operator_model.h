#pragma once
#include <iostream>
#include <despot/util/random.h>
#include "libgeometry.h"

class OperatorModel {
private:
    enum { NO_RISK = 0, RISK = 1, NONE = 2};
    std::map<std::string, PerceptionPerformance> _performance;

public:
    OperatorModel();
    OperatorModel(const std::map<std::string, PerceptionPerformance> &perception_performance);
    ~OperatorModel();
	
    double intAcc(const int time, const std::string type) const;
    int execIntervention(const int time, const bool risk, const std::string type) const;
    int execIntervention(const int time, const bool risk, const double rand_num, const std::string type) const;
    
};
