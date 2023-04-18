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
		   "req_time: " + to_string(req_time) + "\n" +
		   "req_target: " + to_string(req_target) + "\n" +
		   "recognition: " + to_string(recognition) + "\n" +
		   "risk_state: " + to_string(risk_state) + "\n";
}

int Ras::NumActions() const {
	return 1+ target_risk.size()*2
}

bool Ras::Step(State& state, double rand_num, ACT_TYPE action, double&reward, OBS_TYPE& obs) const {
	RasState& ras_state = static_cast <RasState&>(state);

