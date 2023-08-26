#include "vehicle_model.h"

VehicleModel::VehicleModel() :
        m_max_speed(11.2),
        m_yield_speed(2.8),
        m_max_accel(0.15*9.8),
        m_max_decel(0.3*9.8),
        m_min_decel(0.2*9.8),
        m_safety_margin(15),
        m_delta_t(1.0) {
        }

VehicleModel::VehicleModel(double max_speed, double yield_speed, double max_accel, double max_decel, double min_decel, double safety_margin, double delta_t) :
        m_max_speed(max_speed),
        m_yield_speed(yield_speed),
        m_max_accel(max_accel),
        m_max_decel(max_decel),
        m_min_decel(min_decel),
        m_safety_margin(safety_margin),
        m_delta_t(delta_t) {
        }

double VehicleModel::getAccel(const double speed, const int pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) const {

    // if (speed <= m_yield_speed) return 0.0;

    std::vector<double> acc_list;
	for (auto itr=recog_list.begin(), end=recog_list.end(); itr!=end; itr++) {
        int distance = target_poses[std::distance(recog_list.begin(), itr)] - pose;
        double max_decel_dist = m_safety_margin + (std::pow(m_yield_speed, 2.0) - std::pow(m_max_speed, 2.0))/(2.0*-m_min_decel);

        if (distance < 0 || max_decel_dist < distance || *itr == false) continue;

        double a = (std::pow(m_yield_speed, 2.0) - std::pow(speed, 2.0))/(2.0*(distance-m_safety_margin));
        // std::cout << "distance: " << distance << ", accel : " << a << std::endl;
        // if (-m_min_decel < a && a < 0.0) continue;

        acc_list.emplace_back(a);
    }

    if (acc_list.empty()) { 
        double a = (m_max_speed - speed)/m_delta_t;
        acc_list.emplace_back(a);
    }

    double min_acc = 1000.0;
    for (const auto itr : acc_list) {
        if (itr < min_acc) {
            min_acc = itr;
        }
    }
    // auto a_itr = std::min_element(acc_list.begin(), acc_list.end());
    // double min_acc = *a_itr;
    // std::cout << "min_acc : " << min_acc << std::endl;
    // int decel_target = target.distanceance(acc_list.begin(), a_itr);
    // std::cout << "pose: " << pose << " speed: " << speed << " acc: " << min_acc << std::endl;
    

    return min_acc;
}


double VehicleModel::clipSpeed(const double acc, const double v0) {

    double speed = v0 + acc * m_delta_t;

    if (acc >= 0.0) {
        if (speed > m_max_speed)
            return m_max_speed - v0;
        else
            return (acc < m_max_accel) ? acc : m_max_accel;
    }
    else {
        if (speed < m_yield_speed)
            return m_yield_speed - v0;
        else
            return (acc > -m_max_decel) ? acc : -m_max_decel;
    }
}

void VehicleModel::getTransition(double& speed, int& pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) const {

    double v0 = speed;
    double a = getAccel(speed, pose, recog_list, target_poses);

    speed = v0 + a * m_delta_t;
    pose += v0 * m_delta_t + 0.5*a*std::pow(m_delta_t, 2.0);

    return;
}
