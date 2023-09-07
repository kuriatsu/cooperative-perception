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
        distance = -100;
    }

    Risk(std::string _id, double _risk_p, Pose _pose) {
        id = _id;
        risk_prob = _risk_p;
        risk_pred = _risk_p >= 0.5;
        risk_hidden = _risk_p >= 0.5;
        pose = _pose;
        distance = -100;
    }        
    
    Risk() {
    }
};

class TAValues {
private:
    int no_action_head = 0;
    int request_head = 1;
    int change_recog_head = 2;
    int max_action_num = 3;

public:
    TAValues() {
    };

    TAValues(int num_targets){
        if (num_targets == 0) {
            no_action_head = 0;
            request_head = 0;
            change_recog_head = 0;
            max_action_num = 1;
        }
        no_action_head = 0;
        request_head = 1;
        change_recog_head = num_targets + 1;
        max_action_num = 1 + num_targets * 2;
    };

    enum OBS {NO_RISK, RISK, NONE};
    enum ACT {NO_ACTION, REQUEST, RECOG};

    int numActions() {
        return max_action_num;
    };

    int getActionTarget(int action) const {
        if (action == no_action_head) {
            return 0;
        }
        else if (request_head <= action && action < change_recog_head) {
            return action - request_head;
        }
        else if (change_recog_head <= action) {
            return action - change_recog_head;
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
        else if (request_head <= action && action < change_recog_head) {
            return REQUEST;
        }
        else if (change_recog_head <= action) {
            return RECOG;
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
        else if (attrib == RECOG) {
            return change_recog_head + target;
        }
        else {
            std::cerr << "action attrib may be out of range \n" <<
                "num_action :" << attrib << 
                "target :" << target << std::endl;
            return no_action_head;
        }
    };


    void printAction(int action, int& target_index, std::ostream& out) {

        if (action == no_action_head) {
            out <<  "NO_ACTION" << std::endl;
        }
        else if (request_head <= action && action < change_recog_head) {
            target_index = action - request_head;
            out << "REQUEST to " << target_index;
        }
        else if (change_recog_head <= action) {
            target_index = action - change_recog_head;
            out << "change RECOG of " << target_index;
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
