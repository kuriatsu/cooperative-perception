#include "ras.h"

#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bound.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot {

RasState::RasState() {
}

RasState::~RasState() {
}

string RasState::text() const {
	return "ego_pose: " + to_string(ego_pose) + "\n" + 
		   "ego_speed: " + to_string(ego_speed) + "\n" +
		   "ego_recog: " + to_string(ego_recog) + "\n" +
		   "req_time: " + to_string(req_time) + "\n" +
		   "req_target: " + to_string(req_target) + "\n" +
		   "target_risk: " + to_string(target_risk) + "\n";
		   "target_pose: " + to_string(target_pose) + "\n";
}

int Ras::NumActions() const {
	return 1+ target_risk.size()*2
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

