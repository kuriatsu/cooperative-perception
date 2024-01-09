#include <iostream>
#include <operator_model.h>

OperatorModel::OperatorModel() {
}

OperatorModel::OperatorModel(const std::map<std::string, PerceptionPerformance> &perception_performance) :
    _performance(perception_performance) {
}

OperatorModel::~OperatorModel(){
}

double OperatorModel::intAcc(const int time, const std::string type) const {

    if (time < _performance.at(type).ope_min_time) {
        return 0.5;
    }
    else {
        double acc = _performance.at(type).ope_min_acc + _performance.at(type).ope_slope_acc_time * (time - _performance.at(type).ope_min_time);
        // std::cout << "acc :" << acc << ", time : " << time << std::endl;
        acc = (acc < _performance.at(type).ope_max_acc) ? acc : _performance.at(type).ope_max_acc;
        return acc;
    }
}

int OperatorModel::execIntervention(const int time, const bool risk, const std::string type) const {

    if (time == 0) {
        return TAValues::RISK;
    }
    else {
        double rand_num = despot::Random::RANDOM.NextDouble();
        double acc = intAcc(time, type);
    
        // TODO: is this output is okay? especially "else" section
        if (rand_num < acc) {
            return (risk == TAValues::RISK) ? TAValues::RISK : TAValues::NO_RISK;    
        }
        else {
            return (risk == TAValues::RISK) ? TAValues::NO_RISK : TAValues::RISK;    
            // return (rand_num < 0.5) ? TAValues::RISK : TAValues::NO_RISK;
        }
    }
}

int OperatorModel::execIntervention(const int time, const bool risk, const double rand_num, const std::string type) const {

    if (time == 0) {
        return TAValues::RISK;
    }
    else {
        double acc = intAcc(time, type);
    
        // TODO: is this output is okay? especially "else" section
        if (rand_num <= acc) {
            return (risk == TAValues::RISK) ? TAValues::RISK : TAValues::NO_RISK;    
        }
        else {
            // std::cout << "acc" << acc << "rand num" << rand_num << std::endl;
            return (risk == TAValues::RISK) ? TAValues::NO_RISK : TAValues::RISK;    
            // return (rand_num < 0.5) ? TAValues::RISK : TAValues::NO_RISK;
        }
    }
}
