#include "operator_model.h"

OperatorModel::OperatorModel(){
	min_time = 3.0;
	acc_time_min = 0.5;
	acc_time_slope = 0.25;
}

OperatorModel::~OperatorModel(){
}

double OperatorModel::int_acc(const int time) const {

	if (time < min_time) {
		return false;
	}
	else {
		double acc = acc_time_min + acc_time_slope * (time - min_time);
		acc = (acc < 1.0) ? acc : 1.0;
		return acc;
	}
}

