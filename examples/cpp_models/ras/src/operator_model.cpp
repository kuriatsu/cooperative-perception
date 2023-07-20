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

int OperatorModel::execIntervention(const int time, const std::string action, const std::string target, const bool risk) {

    if (action == "RECOG" || action == "NO_ACTION") {
        last_target_id = "NONE";
        return NONE;
    }
    else {
        double rand_num = Random::RANDOM.NextDouble();
        double acc = int_acc(time);
    
        if (last_target_id == target) {
            // TODO: is this output is okay? especially "else" section
            if (rand_num < acc) {
                return risk ? RISK : NO_RISK;    
            }
            else {
                return risk ? NO_RISK : RISK;
            }
        }
        else {
            last_target_id = target;
            return NONE
        }
    }
}
