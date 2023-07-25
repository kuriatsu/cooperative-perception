#include <iostream>
#include <operator_model.h>

OperatorModel::~OperatorModel(){
}

double OperatorModel::int_acc(const int time) const {

	if (time < min_time) {
		return 0.5;
	}
	else {
		double acc = acc_time_min + acc_time_slope * (time - min_time);
        // std::cout << acc_time_min << "," << acc_time_slope << time << min_time << std::endl;
		acc = (acc < 1.0) ? acc : 1.0;
		return acc;
	}
}

int OperatorModel::execIntervention(const int time, const int action, const std::string target, const bool risk) const {

    if (action == TAValues::RECOG || action == TAValues::NO_ACTION) {
        last_target_id = "NONE";
        return TAValues::NONE;
    }
    else {
        double rand_num = despot::Random::RANDOM.NextDouble();
        double acc = int_acc(time);
    
        if (last_target_id == target) {
            // TODO: is this output is okay? especially "else" section
            if (rand_num < acc) {
                return risk ? TAValues::RISK : TAValues::NO_RISK;    
            }
            else {
                return risk ? TAValues::NO_RISK : TAValues::RISK;
            }
        }
        else {
            last_target_id = target;
            return TAValues::NONE;
        }
    }
}
