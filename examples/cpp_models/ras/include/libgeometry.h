#pragma once

#include <iostream>
#include <libsumo/libtraci.h>
#include <math.h>

using namespace libtraci;

struct PerceptionPerformance {
    double ope_min_time;
    double ope_min_acc;
    double ope_slope_acc_time;
    double ope_max_acc;
    double ads_mean_acc;
    double ads_dev_acc;
};

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

    Pose(libsumo::TraCIPosition position) {
        x = position.x;
        y = position.y;
        theta = 0.0;
    }

    Pose(std::string id, std::string attrib) {
        libsumo::TraCIPosition position;
        double angle;

        if (attrib == "v" || attrib == "vehicle") {
            position = Vehicle::getPosition(id);
            angle = M_PI*Vehicle::getAngle(id)/180.0;
            lane = Vehicle::getRoadID(id);
            lane_position = Vehicle::getLanePosition(id);
        }
        else if (attrib == "p" || attrib == "person") {
            position = Person::getPosition(id);
            angle = M_PI*Person::getAngle(id)/180.0;
            lane = Person::getRoadID(id);
            lane_position = Person::getLanePosition(id); 
        }
        x = position.x;
        y = position.y;
        theta = angle;
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
    std::string type;

public:
    Risk(std::string id_, bool true_risk_, double p_risk_, std::string type_) {
        id = id_;
        risk_prob = p_risk_;
        risk_pred = risk_prob >= 0.5;
        risk_hidden = true_risk_; 
        distance = -100;
        type = type_;
    }

    Risk(std::string id_, bool true_risk_, double p_risk_, Pose pose_, std::string type_){
        id = id_;
        risk_prob = p_risk_;
        risk_pred = p_risk_ >= 0.5;
        risk_hidden = true_risk_;
        pose = pose_;
        distance = -100;
        type = type_;
    }        
    
    Risk() {
    }
};

class TAValues {
private:
    int no_action_head = 0;
    int request_head = 1;
    int max_action_num = 2;

public:
    TAValues() {
    };

    TAValues(int num_targets){
        if (num_targets == 0) {
            no_action_head = 0;
            request_head = 0;
            max_action_num = 1;
        }
        no_action_head = 0;
        request_head = 1;
        max_action_num = 1 + num_targets;
    };

    enum OBS {NO_RISK, RISK};
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
        else {
            std::cout << "obs value is out of range" << std::endl;
        }
    }
};
