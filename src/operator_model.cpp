#include <iostream>
#include "cooperative_perception/operator_model.hpp"

OperatorModel::~OperatorModel(){
}

double OperatorModel::InterventionAccuracy(const int time) const {

	if (time < _min_time) {
		return 0.5;
	}
	else {
		double acc = _acc_time_min + _acc_time_slope * (time - _min_time);
        // std::cout << "acc :" << acc << ", time : " << time << std::endl;
		acc = (acc < 1.0) ? acc : 1.0;
		return acc;
	}
}

int OperatorModel::ExecIntervention(const int time, const int action, const std::string target, const bool risk) const {

    if (action == CPValues::NO_ACTION) {
        _last_target_id = "NONE";
        return CPValues::NONE;
    }
    else {
        double rand_num = despot::Random::RANDOM.NextDouble();
        double acc = InterventionAccuracy(time);
        _last_target_id = target;
    
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
