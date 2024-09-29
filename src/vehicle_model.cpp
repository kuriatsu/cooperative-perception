#include "cooperative_perception/vehicle_model.hpp"

VehicleModel::VehicleModel() :
        max_speed_(11.2),
        yield_speed_(2.8),
        max_accel_(0.15*9.8),
        max_decel_(0.3*9.8),
        min_decel_(0.2*9.8),
        safety_margin_(5),
        delta_t_(1.0) {
        }

VehicleModel::VehicleModel(const double delta_t):
        delta_t_(delta_t) {
        }

VehicleModel::VehicleModel(double max_speed, double yield_speed, double max_accel, double max_decel, double min_decel, double safety_margin, double delta_t) :
        max_speed_(max_speed_),
        yield_speed_(yield_speed_),
        max_accel_(max_accel_),
        max_decel_(max_decel_),
        min_decel_(min_decel_),
        safety_margin_(safety_margin_),
        delta_t_(delta_t_) {
        }


double VehicleModel::GetDecelDistance(const double speed, const double acc, const double safety_margin) const {
    double decel = (acc > 0.0) ? -acc : acc;
    return safety_margin_ + (std::pow(yield_speed_, 2.0) - std::pow(speed, 2.0))/(2.0*decel);
}

double VehicleModel::GetDecelTime(const double speed, const double acc) const {
    double decel = (acc > 0.0) ? -acc : acc;
    return speed / decel;
}

double VehicleModel::GetAccel(const double speed, const int pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) const {

    // if (speed <= yield_speed_) return 0.0;

    std::vector<double> acc_list;
	for (auto itr=recog_list.begin(), end=recog_list.end(); itr!=end; itr++) {
        int distance = target_poses[std::distance(recog_list.begin(), itr)] - pose;
        double emergency_decel_dist = GetDecelDistance(speed, max_decel_, 0.0);
        double comf_decel_dist = GetDecelDistance(speed, min_decel_, safety_margin_);

        // std::cout << pose << " decel_dist" << emergency_decel_dist << " comf_decel" << comf_decel_dist  << " distance: " << distance << std::endl;
        // if (distance < 0 || *itr == false) continue;
        // if (distance < 0 || emergency_decel_dist > distance || *itr == false) continue;
        if (distance < 0 || comf_decel_dist + 10 < distance || *itr == false) continue;
        if (distance > comf_decel_dist) {
            double a = (std::pow(yield_speed_, 2.0) - std::pow(speed, 2.0))/(2.0*(distance+safety_margin_));
            acc_list.emplace_back(a);
        }
        else if (distance > emergency_decel_dist) {
            double a = (std::pow(yield_speed_, 2.0) - std::pow(speed, 2.0))/(2.0*(distance));
            acc_list.emplace_back(a);
        }
        else {
            acc_list.emplace_back(-max_decel_);
        }

        // std::cout << pose << " decel_dist" << emergency_decel_dist << " comf_decel" << comf_decel_dist  << " distance: " << distance << ", accel : " << a << std::endl;
        // if (-min_decel_ < a && a < 0.0) continue;

    }

    if (speed < max_speed_ || speed < yield_speed_) 
        acc_list.emplace_back(max_accel_);

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


double VehicleModel::ClipSpeed(const double acc, const double v0) const{

    double clipped_acc=acc;

    // std::cout << "acc : " << acc << " : max_decel : " << max_decel_ << " v0 : " << v0 << " yield: " << yield_speed_ << std::endl;
    if (acc >= 0.0 && acc > max_accel_)
        clipped_acc = max_accel_;
    else if (acc < 0.0 && acc < -max_decel_)
        clipped_acc = -max_decel_;

    // double speed = v0 + clipped_acc * delta_t_; 
    double speed = v0 + clipped_acc; 

    if (speed >= max_speed_)
        clipped_acc = (max_speed_ - v0);
        // clipped_acc = (max_speed_ - v0) / delta_t_;

    else if (speed <= yield_speed_)
        // clipped_acc = (yield_speed_ - v0) / delta_t_;
        clipped_acc = (yield_speed_ - v0);

    // std::cout << acc << clipped_acc << std::endl;
    return clipped_acc;
}

void VehicleModel::GetTransition(double& speed, int& pose, const std::vector<bool>& recog_list, const std::vector<int>& target_poses) const {

    for (int i = 0; i < delta_t_; i++) {
        double v0 = speed;
        double a = GetAccel(speed, pose, recog_list, target_poses);
        double clipped_a = ClipSpeed(a, v0);

        // speed = v0 + clipped_a * delta_t_;
        // pose += v0 * delta_t_ + 0.5*clipped_a*std::pow(delta_t_, 2.0);
        speed = v0 + clipped_a;
        pose += v0 + 0.5 * clipped_a;
    }

    return;
}
