#include "operator_model.h"
#include <iostream>

OperatorModel::OperatorModel(){
	min_time = 3.0;
	acc_time_min = 0.5;
	acc_time_slope = 0.25;
}

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

