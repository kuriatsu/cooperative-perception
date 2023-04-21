#include "ras.h"

#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bound.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot {

RasState::RasState() {
	ego_pose = 0;
	ego_speed = 11.2;
	req_time = 0;
	req_target = NO_TARGET;

	bool recog[] = {NO_RISK, RISK};
	bool risk[] = {NO_RISK, RISK};
	int poses[] = {80, 100};

	ego_recog.assign(std::begin(recog), std::end(ecog));
	target_risk.assign(std::begin(risk), std::end(risk));
	target_pose.assing(std::begin(poses), std::end(poses));
}

RasState::~RasState() {
}

string RasState::text() const {
	return "ego_pose: " + to_string(ego_pose) + "\n" + 
		   "ego_speed: " + to_string(ego_speed) + "\n" +
		   "ego_recog: " + to_string(ego_recog) + "\n" +
		   "req_time: " + to_string(req_time) + "\n" +
		   "req_target: " + to_string(req_target) + "\n" +
		   "target_risk: " + to_string(target_risk) + "\n" +
		   "target_pose: " + to_string(target_pose) + "\n";
}

Ras::Ras() {
	target_num = 2;
	min_speed = 2.8;
	ordinary_G = 0.2;
	safety_margin = 5;
}

int Ras::NumActions() const {
	return 1 + target_num * 2;
}

bool Ras::Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const {
	RasState& ras_state = static_cast <RasState&>(state);
	reward = 0.0;

	// ego state trantion
	std::vector<int> ego_state1;// pose, speed
	ego_state1.resize(2);
	ego_transition(ras_state.ego_pose, ras_state.ego_speed, ras_state.recog, action, ego_state1, reward);
	state.ego_pose = ego_state1[0];	
	state.ego_speed = ego_state1[1];	

	// ego_recog, request, observation
	if (RECOG <= action < NO_ACTION) {
		state.ego_recog[action-RECOG] *= -1;
		obs = NO_INT;
	}
	else if (REQUEST <= action < RECOG) {
		int index = action - REQUEST;
		double acc = intervention_acc(state.req_time);

		if (state.req_target == index) {
			obs = (rand_num > acc & state.risk_state[index] != state.ego_recog[index]): INT : NO_INT;
			state.req_time++;
		} 
		else {
			state.req_time = 1;
			state.req_target = index;
			obs = NO_INT;
		}
	}

	if (ras_state.ego_pose >= 150)
		return true;
	else
		return false;
}


void Ras::EgoVehicleTransition(const int& pose, const int& speed, const vector<int>& recog_list, const vector<int>& target_list, const ACT_TYPE& action, vector<int>& state_after, int& reward){
	std::vector<double> acc_list;

	if speed < self.ideal_speed {
		acc_list.emplace_back(ordinary_G);
	}
	else if {
		acc_list.emplace_back(0.0);
	}
	else {
		acc_list.emplace_back(-ordinary_G);
	}

	for (auto it=recog_list.begin(), end=recog_list.end(); it != end; ++it) {
		target_position = target_list[std::distance(recog_list.begin(), it)];
        int dist = target_position - pose;
        bool is_decel_target = false; 

        if (dist < 0) {
            is_decel_target = false;
        }
        else if (action == INT) {
            is decel_target = (*it == true);
        }
        else {
            is_decel_target = (*it == true)
        }

        if (!is_decel_target){
            continue;
        }

        double a = 0.0;
        decel_distance = (speed** - min_speed**2)/(2*9.8*ordinary_G) + safety_margin;

        if (dist > decel_distance) {
            a = (min_speed**2-speed**2)/(2*(dist-safety_margin));
        }
        else {
            a = 0.0;
        }

        acc_list.emplace_back(a);
    }

    a_itr = std::min_element(acc_list.begin(), acc_list.end());
    a = acc_list(a_itr);
    decel_target = std::distance(acc_list.begin(), a_itr);
    auto &x = state_after[0];
    auto &v = state_after[1];
    v = speed + a*delta_t;
    if (v <= min_speed) {
        v = min_speed;
        a = 0.0;
    }
    else if (v >= ideal_speed) {
        v = ideal_speed;
        a = 0.0;
    }

    x = pose + speed * delta_t + 0.5*a*delta_t**2;
    return;
}

double Ras::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {
    const SimpleState& simple_state = static_cast<const SimpleState&>(state);

    if (REQUEST <= action & action < RECOG) {
        return operator_model::int_prob(state.req_time);
    }
    else {
        return 1.0;
    }
}


