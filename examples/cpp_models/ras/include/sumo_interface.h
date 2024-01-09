#pragma once

#include <iostream>
#include <libsumo/libtraci.h>
#include <unordered_map>
#include <random>
#include <algorithm>
#include "libgeometry.h"
#include "vehicle_model.h"

using namespace libtraci;

class SumoInterface {

private:
    double _delta_t;
    double _density; // 1ppl per 1m
    std::vector<double> _perception_range; // x l&r, y_forward

    VehicleModel *_vehicle_model;
    std::string _ego_name = "ego_vehicle";

    std::unordered_map<std::string, Risk> _risks;

    // for logging // deprecated
    std::vector<std::string> _passed_targets;
    std::map<std::string, double> _obstacle_type_rate;

public:
    std::map<std::string, PerceptionPerformance> _perception_performance;

public:

    SumoInterface();
    SumoInterface(VehicleModel *vehicle_model, double delta_t, double density, std::vector<double> perception_range, const std::map<std::string, PerceptionPerformance> &perception_performance, std::map<std::string, double> obstacle_type_rate);

    std::vector<std::string> perception();
    void controlEgoVehicle(const std::vector<int>& target_poses, const std::vector<bool> target_risks) const;
    void spawnPedestrians();
    void spawnPedestrians(std::vector<Risk> obj_list); 
    void spawnEgoVehicle();
    double getEgoSpeed();
    Risk* getRisk(const std::string& id);
    std::vector<Risk> getRisk(const std::vector<std::string>& ids);
    void step(int delta_t = 0);
    void Run();
    void start();
    void close();
    void setColor(const std::string id, const std::vector<int> color, const std::string attrib) const;
    void log(double& time, Pose& ego_pose, std::vector<double>& other_ego_info, std::vector<Risk>& log_risks);
    bool isTerminate();
    

};
