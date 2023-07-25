#pragma once

#include <iostream>
#include <libsumo/libtraci.h>
#include <math.h>

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
    double risk_prob;
    bool risk_pred;
    bool risk_hidden;
    Pose pose;
    double distance;

public:
    Risk(std::string _id, double _risk_p) {
        id = _id;
        risk_prob = _risk_p;
        risk_pred = _risk_p >= 0.5;
        risk_hidden = _risk_p >= 0.5;
    }

    Risk(std::string _id, double _risk_p, Pose _pose) {
        id = _id;
        risk_prob = _risk_p;
        risk_pred = _risk_p >= 0.5;
        risk_hidden = _risk_p >= 0.5;
        pose = _pose;
    }        
    
    Risk() {
    }
};

class TAValues {
private:
    int request_head = 0;
    int no_action_head = 1;
    int change_recog_head = 2;
    int max_action_num = 2;

public:
    TAValues() {
    };

    TAValues(int num_targets){
        if (num_targets == 0) {
            request_head = 0;
            no_action_head = 0;
            change_recog_head = 0;
            max_action_num = 1;
        }
        request_head = 0;
        no_action_head = num_targets;
        change_recog_head = num_targets + 1;
        max_action_num = 1 + num_targets * 2;
    };

    enum OBS {NO_RISK, RISK, NONE};
    enum ACT {REQUEST, NO_ACTION, RECOG};

    int numActions() {
        return max_action_num;
    };

    ACT getActionTarget(int action, int& target_index) const {
        if (request_head <= action && action < no_action_head) {
            target_index = action - request_head;
            return REQUEST;
        }
        else if (action == no_action_head) {
            target_index = 0;
            return NO_ACTION;
        }
        else if (change_recog_head <= action) {
            target_index = action - change_recog_head;
            return RECOG;
        }
        else {
            std::cout << "action index may be out of range \n" <<
                "num_action :" << max_action_num << "\n" << 
                "action :" << action << std::endl;
            return NO_ACTION;
        }
    };

};
