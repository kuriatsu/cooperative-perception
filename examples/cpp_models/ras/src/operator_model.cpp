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

