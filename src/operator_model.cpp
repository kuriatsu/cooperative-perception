#include <iostream>
#include "cooperative_perception/operator_model.hpp"

OperatorModel::OperatorModel() {
}

OperatorModel::OperatorModel(const std::map<std::string, PerceptionPerformance> &performance) :
    performance_(performance) {
}

OperatorModel::~OperatorModel(){
}

double OperatorModel::InterventionAccuracy(const int time, const std::string type) const {

    if (time < performance_.at(type).ope_min_time) {
        return 0.5;
    }
    else {
        double acc = performance_.at(type).ope_min_acc + performance_.at(type).ope_slope_acc_time * (time - performance_.at(type).ope_min_time);
        // std::cout << "acc :" << acc << ", time : " << time << std::endl;
        acc = (acc < performance_.at(type).ope_max_acc) ? acc : performance_.at(type).ope_max_acc;
        return acc;
    }
}

int OperatorModel::ExecIntervention(const int time, const bool risk, const std::string type) const {

    if (time == 0) {
        return CPValues::RISK;
    }
    else {
        double rand_num = despot::Random::RANDOM.NextDouble();
        double acc = InterventionAccuracy(time, type);
    
        // TODO: is this output is okay? especially "else" section
        if (rand_num < acc) {
            return (risk == CPValues::RISK) ? CPValues::RISK : CPValues::NO_RISK;    
        }
        else {
            return (risk == CPValues::RISK) ? CPValues::NO_RISK : CPValues::RISK;    
            // return (rand_num < 0.5) ? CPValues::RISK : CPValues::NO_RISK;
        }
    }
}

int OperatorModel::ExecIntervention(const int time, const bool risk, const double rand_num, const std::string type) const {

    if (time == 0) {
        return CPValues::RISK;
    }
    else {
        double acc = InterventionAccuracy(time, type);
    
        // TODO: is this output is okay? especially "else" section
        if (rand_num <= acc) {
            return (risk == CPValues::RISK) ? CPValues::RISK : CPValues::NO_RISK;    
        }
        else {
            // std::cout << "acc" << acc << "rand num" << rand_num << std::endl;
            return (risk == CPValues::RISK) ? CPValues::NO_RISK : CPValues::RISK;    
            // return (rand_num < 0.5) ? CPValues::RISK : CPValues::NO_RISK;
        }
    }
}
