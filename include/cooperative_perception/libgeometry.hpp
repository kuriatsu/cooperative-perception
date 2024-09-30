#pragma once

#include <iostream>
#include <math.h>
#include "despot/interface/pomdp.h"

namespace despot {

class CPState : public State {
public:
	int ego_pose;
    double ego_speed;
	std::vector<bool> ego_recog;
	int req_time;
	int req_target;
    std::vector<int> risk_pose;

	// hidden state
	std::vector<bool> risk_bin;
    
    CPState() : State() {
        ego_pose = 0;
        ego_speed = 11.2;
        req_time = 0;
        req_target = 0;
        // ego_recog = {false, true, true};
        // risk_pose = {80, 100, 120};
        // risk_bin = {true, true, false};
        ego_recog = {false, false, true};
        risk_pose = {60, 100, 120};
        risk_bin = {false, false, true};
    }

    CPState(int _ego_pose, float _ego_speed, std::vector<bool> _ego_recog, int _req_time, int _req_target, std::vector<bool> _risk_bin, std::vector<int> _risk_pose) :
            ego_pose(_ego_pose),
            ego_speed(_ego_speed),
            ego_recog(_ego_recog),
            req_time(_req_time),
            req_target(_req_target),
            risk_pose(_risk_pose),
            risk_bin(_risk_bin)	{
    }

    ~CPState() {
    }

    std::string text() const {
        return "ego_pose: " + to_string(ego_pose) + "\n" + 
               "ego_speed: " + to_string(ego_speed) + "\n" + 
               "ego_recog: " + to_string(ego_recog) + "\n" +
               "req_time: " + to_string(req_time) + "\n" +
               "req_target: " + to_string(req_target) + "\n" +
               "risk_pose: " + to_string(risk_pose) + "\n" +
               "risk_bin: " + to_string(risk_bin) + "\n"; 
        // return "";
    }
};

}


class Pose {
public:
    double x, y;
    double theta;
    double lane_position;
    std::string lane;

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


    Pose() {
    }

    Pose transformTo(const Pose& origin) {
    // transform target position to ego vehicle coordinage
    double s_x, s_y, r_x, r_y;
    s_x = x - origin.x;
    s_y = y - origin.y;
    r_x = s_x * cos(origin.theta) - s_y * sin(origin.theta); 
    r_y = s_x * sin(origin.theta) + s_y * cos(origin.theta); 
    Pose out_pose(r_x, r_y);
    return out_pose; 
}
};


class Risk {
public:
    std::string id;
    double risk_prob;
    bool risk_pred;
    bool risk_hidden;
    Pose pose;
    double distance;

public:
    Risk(std::string id_, bool risk_val_, double p_risk_) {
        id = id_;
        risk_prob = p_risk_;
        risk_pred = risk_prob >= 0.5;
        risk_hidden = risk_val_; 
        distance = -100;
    }

    Risk(std::string id_, bool risk_val_, double p_risk_, Pose pose_) {
        id = id_;
        risk_prob = p_risk_;
        risk_pred = p_risk_ >= 0.5;
        risk_hidden = risk_val_;
        pose = pose_;
        distance = -100;
    }        
    
    Risk() {
    }
};

class CPValues {
private:
    int no_action_head = 0;
    int request_head = 1;
    int max_action_num = 2;

public:
    CPValues() {
    };

    CPValues(int num_targets){
        if (num_targets == 0) {
            no_action_head = 0;
            request_head = 0;
            max_action_num = 1;
        }
        no_action_head = 0;
        request_head = 1;
        max_action_num = 1 + num_targets;
    };

    enum OBS {NONE, NO_RISK, RISK};
    enum ACT {NO_ACTION, REQUEST};

    int numActions() {
        return max_action_num;
    };

    int getActionTarget(int action) const {
        if (action == no_action_head) {
            return 0;
        }
        else if (request_head <= action) {
            return action - request_head;
        }
        else {
            std::cerr << "action index may be out of range \n" <<
                "num_action :" << max_action_num << "\n" << 
                "action :" << action << std::endl;
            return 0;
        }
    };

    ACT getActionAttrib(int action) const {
        if (action == no_action_head) {
            return NO_ACTION;
        }
        else if (request_head <= action) {
            return REQUEST;
        }
        else {
            std::cerr << "action index may be out of range \n" <<
                "num_action :" << max_action_num << "\n" << 
                "action :" << action << std::endl;
            return NO_ACTION;
        }
    };

    int getAction(ACT attrib, int target) const {
        if (attrib == NO_ACTION) {
            return no_action_head;
        }
        else if (attrib == REQUEST) {
            return request_head + target;
        }
        else {
            std::cerr << "action attrib may be out of range \n" <<
                "num_action :" << attrib << 
                "target :" << target << std::endl;
            return no_action_head;
        }
    };

    std::string getActionName(int action) const {
        if (action == no_action_head) {
            return "NO_ACTION";
        }
        else if (request_head <= action) {
            return "REQUEST";
        }
        else {
            std::cerr << "action index may be out of range \n" <<
                "num_action :" << max_action_num << "\n" << 
                "action :" << action << std::endl;
            return "ERR";
        }
    };

    std::string getObsName(int obs) {
        if (obs == NO_RISK) {
            return "NO_RISK";
        }
        else if (obs == RISK) {
            return "RISK";
        }
        else if (obs == NONE) {
            return "NONE";
        }
        else {
            std::cerr << "obs value is out of range" << std::endl;
            return "ERR";
        }
    };

    void printAction(int action, int& target_index, std::ostream& out) {

        if (action == no_action_head) {
            out <<  "NO_ACTION" << std::endl;
        }
        else if (request_head <= action) {
            target_index = action - request_head;
            out << "REQUEST to " << target_index;
        }
        else {
            std::cerr << "action index may be out of range \n" <<
                "num_action :" << max_action_num << "\n" << 
                "action :" << action << std::endl;
            out << "NO_ACTION" << std::endl;
        }
    };

    void printObs(int obs) {
        if (obs == NO_RISK) {
            std::cout << "obs : NO_RISK" << std::endl;
        }
        else if (obs == RISK) {
            std::cout << "obs : RISK" << std::endl;
        }
        else if (obs == NONE) {
            std::cout << "obs : NONE" << std::endl;
        }
        else {
            std::cout << "obs value is out of range" << std::endl;
        }
    };
};
