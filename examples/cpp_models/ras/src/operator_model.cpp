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
        // std::cout << "acc :" << acc << ", time : " << time << std::endl;
		acc = (acc < 1.0) ? acc : 1.0;
		return acc;
	}
}

int OperatorModel::execIntervention(const int time, const bool risk) const {

    if (time == 0) {
        return TAValues::RISK;
    }
    else {
        double rand_num = despot::Random::RANDOM.NextDouble();
        double acc = int_acc(time);
    
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
