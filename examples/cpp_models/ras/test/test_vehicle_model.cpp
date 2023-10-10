#include "vehicle_model.h"

int main(void) {
    std::vector<bool> recog_list = {true, true};
    std::vector<int> target_poses = {100, 110};
    VehicleModel vehicle_model;
    vehicle_model.m_delta_t = 1.0;
    int ego_pose = 0;
    double ego_speed = 11.2;

    while (ego_pose < 150) {
        std::cout << ego_pose << ": " << ego_speed << " emergency : " << vehicle_model.getDecelDistance(ego_speed, vehicle_model.m_max_decel, vehicle_model.m_safety_margin) << " comf: " << vehicle_model.getDecelDistance(ego_speed, vehicle_model.m_min_decel, vehicle_model.m_safety_margin) << std::endl;
        vehicle_model.getTransition(ego_speed, ego_pose, recog_list, target_poses);
    }
}
