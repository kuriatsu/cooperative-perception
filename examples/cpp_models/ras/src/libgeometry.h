#pragma once

#include <iostream>
#include <libsumo/libtraci.h>

class Pose {
public:
    double x, y;
    double theta;

public:
    Pose(double _x, double _y, double _theta) {
        x = _x;
        y = _y;
        theta = _theta;
    }

    Pose(double _x, double _y) {
        x = _x;
        y = _y;
        theta = 0.0;
    }

    Pose(std::vector<double> vec) {
        x = vec[0];
        y = vec[1];
        theta = 0.0;
    }

    Pose(libsumo::TraCIPosition position) {
        x = position.x;
        y = position.y;
        theta = 0.0;
    }

    Pose() {
    }

    Pose transformTo(const Pose& origin);

};


Pose Pose::transformTo(const Pose& origin) {
    // transform target position to ego vehicle coordinage
    double s_x, s_y, r_x, r_y;
    s_x = x - origin.x;
    s_y = y - origin.y;
    r_x = s_x * cos(origin.theta) - s_y * sin(origin.theta); 
    r_y = s_x * sin(origin.theta) + s_y * cos(origin.theta); 
    Pose out_pose(r_x, r_y);
    return out_pose; 
}

class Risk {
public:
    std::string id;
    double p_risk;
    bool risk;
    Pose pose;
    double distance;

public:
    Risk(std::string _id, double _p_risk) {
        id = _id;
        p_risk = _p_risk;
        risk = p_risk > 0.5;
    }

    Risk(std::string _id, double _p_risk, Pose _pose) {
        id = _id;
        p_risk = _p_risk;
        risk = p_risk > 0.5;
        pose = _pose;
    }        
    
    Risk() {
    }
};
