#include "vehicle_model.h"

double VehicleModel::getAccel(const double speed, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) {

    if (speed < m_yield_speed) return 0.0;

    std::vector<double> acc_list;
	for (auto itr=recog_list.begin(), end=recog_list.end(); itr!=end; itr++) {
        int target_pose = target_poses[std::distance(recog_list.begin(), itr)];

        if (target_pose < 0 || *it == false) continue;

        double a = (pow(m_yield_speed, 2.0) - pow(speed, 2.0))/(2.0*(target_pose-m_safety_margin));
        std::cout << "ped: " << target.id << " dist: "<<  target.distance << " acc: " << a << std::endl;
        acc_list.emplace_back(a);
    }

    if (acc_list.empty()) { 
        double a = (m_max_speed - speed)/m_delta_t;
        acc_list.emplace_back(a);
    }

    auto a_itr = min_element(acc_list.begin(), acc_list.end());
    double min_acc = *a_itr;
    // int decel_target = target.distanceance(acc_list.begin(), a_itr);
    std::cout << "speed: " << speed << " acc: " << min_acc << std::endl;
    
    if (min_acc >= 0.0) {
        min_acc = std::min(min_acc, m_v_max_accel);
    }
    else {
        min_acc = std::max(min_acc, m_v_max_decel);
    }

    return min_acc;
}


double VehicleModel::getTransition(double& speed, int& pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) {

    double a = getAccel(speed, recog_list, target_poses);
    // int decel_target = distance(acc_list.begin(), a_itr);
    speed += a * m_delta_t;
    if (speed <= m_yield_speed) {
        speed =	m_yield_speed;
        // a = 0.0;
    }
    else if (speed >= m_max_speed) {
        speed = m_max_speed;
        // a = 0.0;
    }

    pose += speed * m_delta_t + pow(0.5*a*m_delta_t, 2.0);
    return;
}
