#include "vehicle_model.h"

VehicleModel::VehicleModel() :
        m_max_speed(11.2),
        m_yield_speed(2.8),
        m_max_accel(0.15*9.8),
        m_max_decel(0.3*9.8),
        m_min_decel(0.2*9.8),
        m_safety_margin(5),
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


double VehicleModel::getDecelDistance(const double speed, const double acc, const double safety_margin) const {
    double decel = (acc > 0.0) ? -acc : acc;
    return safety_margin + (std::pow(m_yield_speed, 2.0) - std::pow(speed, 2.0))/(2.0*decel);
}

double VehicleModel::getDecelTime(const double speed, const double acc) const {
    double decel = (acc > 0.0) ? -acc : acc;
    return speed / decel;
}

double VehicleModel::getAccel(const double speed, const int pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) const {

    // if (speed <= m_yield_speed) return 0.0;

    std::vector<double> acc_list;
	for (auto itr=recog_list.begin(), end=recog_list.end(); itr!=end; itr++) {
        int distance = target_poses[std::distance(recog_list.begin(), itr)] - pose;
        double emergency_decel_dist = getDecelDistance(speed, m_max_decel, m_safety_margin);
        double comf_decel_dist = getDecelDistance(speed, m_min_decel, m_safety_margin);

        // std::cout << pose << " decel_dist" << emergency_decel_dist << " comf_decel" << comf_decel_dist  << " distance: " << distance << std::endl;
        // if (distance < 0 || *itr == false) continue;
        // if (distance < 0 || emergency_decel_dist > distance || *itr == false) continue;
        if (distance < 0 || comf_decel_dist + 20 < distance || *itr == false) continue;
        if (distance > comf_decel_dist) {
            double a = (std::pow(m_yield_speed, 2.0) - std::pow(speed, 2.0))/(2.0*(distance+m_safety_margin));
            acc_list.emplace_back(a);
        }
        else if (distance > emergency_decel_dist) {
            double a = (std::pow(m_yield_speed, 2.0) - std::pow(speed, 2.0))/(2.0*(distance));
            acc_list.emplace_back(a);
        }
        else {
            acc_list.emplace_back(-m_max_decel);
        }

        // std::cout << pose << " decel_dist" << emergency_decel_dist << " comf_decel" << comf_decel_dist  << " distance: " << distance << ", accel : " << a << std::endl;
        // if (-m_min_decel < a && a < 0.0) continue;

    }

    if (speed < m_max_speed || speed < m_yield_speed) 
        acc_list.emplace_back(m_max_accel);

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


double VehicleModel::clipSpeed(const double acc, const double v0) const{

    double clipped_acc=acc;

    // std::cout << "acc : " << acc << " : max_decel : " << m_max_decel << " v0 : " << v0 << " yield: " << m_yield_speed << std::endl;
    if (acc >= 0.0 && acc > m_max_accel)
        clipped_acc = m_max_accel;
    else if (acc < 0.0 && acc < -m_max_decel)
        clipped_acc = -m_max_decel;

    // double speed = v0 + clipped_acc * m_delta_t; 
    double speed = v0 + clipped_acc; 

    if (speed >= m_max_speed)
        clipped_acc = (m_max_speed - v0);
        // clipped_acc = (m_max_speed - v0) / m_delta_t;

    else if (speed <= m_yield_speed)
        // clipped_acc = (m_yield_speed - v0) / m_delta_t;
        clipped_acc = (m_yield_speed - v0);

    // std::cout << acc << clipped_acc << std::endl;
    return clipped_acc;
}

void VehicleModel::getTransition(double& speed, int& pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) const {

    for (int i = 0; i < m_delta_t; i++) {
        double v0 = speed;
        double a = getAccel(speed, pose, recog_list, target_poses);
        double clipped_a = clipSpeed(a, v0);

        // speed = v0 + clipped_a * m_delta_t;
        // pose += v0 * m_delta_t + 0.5*clipped_a*std::pow(m_delta_t, 2.0);
        speed = v0 + clipped_a;
        pose += v0 + 0.5 * clipped_a;
    }

    return;
}
