#include "ras.h"

#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
// #include <despot/core/builtin_upper_bound.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot {

RasState::RasState() {
	ego_pose = 0;
	ego_speed = 11.2;
	std::vector<bool> _ego_recog{RISK, NO_RISK};
	ego_recog = _ego_recog;
	req_time = 0;
	req_target = NO_TARGET;
	std::veclt<bool> _risk_bin{RISK, RISK};
	risk_bin = _risk_bin;
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
	planning_horizon = 150;
	ideal_speed = 11.2
	yield_speed = 2.8;
	ordinary_G = 0.2;
	safety_margin = 5;

	std::vector<double> _risk_recog{0.5, 0.2};
	std::vector<int> _risk_pose{80, 100};
	risk_recog = _risk_recog;
	risk_pose = _risk_pose;
	risk_thresh = 0.5;

	r_false_positive = -50;
	r_false_negative = -1000;
	r_eff = -10;
	r_comf = -1;
	r_reqest = -1;
}

int Ras::NumActions() const {
	return 1 + target_num * 2;
}

bool Ras::Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const {
	RasState& state_prev = static_cast <RasState&>(state);
	RasState state_curr = state_prev
	reward = 0.0;

	// ego state trantion
	ego_transition(state_curr.ego_pose, state_curr.ego_speed, state_prev.ego_recog, state_prev.target_pose, action);

	// when action = change recog state 
	if (RECOG <= action < NO_ACTION) {
		int t_index = action - RECOG 
		state_curr.ego_recog[t_index] = !state_prev.ego_recog[t_index];
		obs = NO_INT;
	}
	// when action = request intervention
	else if (REQUEST <= action < RECOG) {
		int t_index = action - REQUEST;
		double acc = intervention_acc(state_prev.req_time);

		// request to the same target
		if (state_prev.req_target == t_index) {
			// observation probability
			obs = (rand_num > acc & state_prev.risk_state[index] != state_prev.ego_recog[index]) ? INT : NO_INT;
			state_curr.req_time ++;
		} 
		// request to new target
		else {
			state_curr.req_time = 1;
			state_curr.req_target = index;
			obs = NO_INT;
		}
	}

	reward = CalcReward(state_prev, state_curr, action);

	if (ras_state.ego_pose >= planning_horizon)
		return true;
	else
		return false;
}

int Ras::CalcReward(const State& state_prev, const State& state_curr const ACT_TYPE& action) const {
	RasState& _state_prev = static_cast<RasState&>(state_prev);
	RasState& _state_curr = static_cast<RasState&>(state_curr);
	int reward = 0;

	for (auto it=_state_prev.target_pose.begin(), end=_state_prev.target_pose.end(); it != end; ++it) {
		target_index = std::distance(_state_prev.target_index.begin(), it);
		if (_state_prev.ego_pose <= *itr < _state_curr.ego_pose) {

			// driving safety
			if (_state_curr.ego_recog[target_index] && !_state_curr.target_risk[target_index]) {
				reward += 1 * r_false_positive;
			}
			else if (!_state_curr.ego_recog[target_index] && _state_curr.target_risk[target_index]) {
				reward += 1 * r_false_negative;
			}

			// driving efficiency
			if (!_state_curr.ego_recog[target_index]) {
				// when no risk, higher is better
				reward += (ideal_speed - _state_curr.ego_speed)/(ideal_speed - yield_speed) * r_eff;
			}
			else {
				// when risk, lower is better
			   	reward += (_state_curr.ego_speed - yield_speed)/(ideal_speed - yield_speed) * r_eff;
			}

		}
	}

	// driving comfort (avoid harsh driving)
	reward += ((_state_curr.ego_speed - _state_prev.ego_speed)/(ideal_speed - yield_speed)) ** 2 * r_comf;
	
	// int request
	if (action != NO_ACTION) {
		reward += 1 * r_request;
	}

	return reward;
}


void Ras::EgoVehicleTransition(int& pose, int& speed, const vector<int>& recog_list, const vector<int>& target_list, const ACT_TYPE& action){
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
        decel_distance = (speed** - yield_speed**2)/(2*9.8*ordinary_G) + safety_margin;

        if (dist > decel_distance) {
            a = (yield_speed**2-speed**2)/(2*(dist-safety_margin));
        }
        else {
            a = 0.0;
        }

        acc_list.emplace_back(a);
    }

    a_itr = std::min_element(acc_list.begin(), acc_list.end());
    a = acc_list(a_itr);
    decel_target = std::distance(acc_list.begin(), a_itr);
    speed += a*delta_t;
    if (speed <= yield_speed) {
        speed =	yield_speed;
        a = 0.0;
    }
    else if (speed >= ideal_speed) {
        speed = ideal_speed;
        a = 0.0;
    }

    pose += speed * delta_t + 0.5*a*delta_t**2;
    return;
}

double Ras::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {
    const SimpleState& simple_state = static_cast<const SimpleState&>(state);

    if (REQUEST <= action & action < RECOG) {
        return operator_model.int_prob(state.req_time);
    }
    else {
        return 1.0;
    }
}

State* Ras::CreateStartState(string type) const {
	
	// set ego_recog and risk_bin based on threshold
	std::vector<bool> _ego_recog, _risk_bin;
	for (auto val : risk_recog) {
		_ego_recog.emplace_back((val < risk_thresh) ? NO_RISK : RISK);
		_risk_bin.emplace_back(RISK);
	}

	return new RasState(
			0, // ego_pose
			ideal_speed, // ego_speed
			0, // req_time
			NO_TARGET, // req_target
			_ego_recog, // ego_recog
			_risk_bin, // target_risk
			)
}

Belief* Ras::InitialBelief(const State* start, string type) const {

	if (type != "DEFAULT" && type != "PARTICLE") {
		std::cout << "specified type " + type + " is not supported";
		exit(1);
	}

	// recognition likelihood of the automated system
	std::vector<std::vector<bool>> risk_bin_list(2**target_num, std::vector<bool>(target_num));
	GetBinProduct(risk_list_bin, 0, 0); 
	vector<State*> particles;

	for (auto row : risk_bin_list) {
		double prob = 1.0;
		std::vector<bool> _ego_recog, _risk_bin;
		// set ego_recog and risk_bin based on threshold
		for (auto col=row.begin(), c_end=row.end(); col!=c_end; col++) {
			idx = std::distance(col.begin(), col);
			_ego_recog.emplace_back((risk_prob[idx] < risk_thresh) ? NO_RISK : RISK);
			if (col) {
				prob *= risk_prob[idx]; 
				_risk_bin.emplace_back(RISK);
			}
			else {
				prob *= 1.0 - risk_prob[idx]; 
				_risk_bin.emplace_back(NO_RISK);
			}
		}

		RasState* p = static_cast<RasState*>(Allocate(-1, prob));  
		p->ego_pose = ego_pose;
		p->ego_speed = ego_speed;
		p->ego_recog = _ego_recog;
		p->req_time = req_time;
	  	p->req_target = req_target;
		p->risk_bin = _risk_bin;
		particles.push_back(p);
	}
	return new ParticleBelief(particles, this);
}


void Ras::GetBinProduct(std::vector<std::vector<bool>>& out_list, int col, int row) {

	if (col == out_list[-1].size()) {
		out_list.emplace_back(buf_list);
		row++;
		return;
	}
	if (row == out_list.size()) {
		return;
	}

	for (int i=0; i<2; i++) {
		GetBinProduct(out_list, col++, row);	
		buf_list[row][col] = (i!=0);
	}
}

double Ras::GetMaxReward() const {
	return 0;
}

ValuedAction Ras::GetBestAction() const {
	return ValuedAction(NONE, 0);
}

State* Ras::Allocate(int state_id, double weight) const {
	RasState* ras_state = memory_pool_.Allocate();
	ras_state->state_id = state_id;
	ras_state->weight = weight;
	return ras_state;
}

State* Ras::Copy(const State* particle) const {
	RasState* state = memory_pool_.Allocate();
	*state = *static_cast<const RasState*>(particle);
	state->SetAllocated();
	return state;
}

void Ras::Free(State* particle) const {
	memory_pool_.Free(static_cast<RasState*>(particle));
}

int Ras::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

void Ras::PrintState(const State& state, ostream& out) const {
	const RasState& ras_state = static_cast<const RasState&>(state);
	out << "ego_pose : " << ras_state.ego_pose << "\n"
		<< "ego_speed : " << ras_state.ego_speed << "\n"
		<< "ego_recog : " << ras_state.ego_recog << "\n"
		<< "req_time : " << ras_state.req_time << "\n"
		<< "req_target : " << ras_state.req_target << "\n"
		<< "risk_bin : " << ras_state.risk_bin << "\n"
		<< endl;
}

void Ras::PrintObs(const State& state, OBS_TYPE obs, ostream& out) cosnt {
	out << (obs ? "INT" : "NO_INT") << endl;
}

void Ras::PrintBelief(const Belief& belief, ostream& out) const {
	const vector<State*>& particles = static_cast<const ParticleBelief&>(belief).particles();
	
	double status = 0;
	vector<double> probs(risk_pose.size());
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		const RasState* state = static_cast<cosnt RasState*>(particle);
		for (auto itr=state.risk_bin.begin(), end=state.risk_bin.end(); itr!=end; itr++) {
			prob[std::distance(state.risk_bin.begin(), itr)] += itr * particle-> weight;
		}
	}

	for (int i = 0; i < risk_pose.size(); i++) {
		out << "risk id : " << i << " prob : " << probs[i] << endl;
	}
}

void Ras::PrintAction(ACT_TYPE action, ostream& out) const {
	if (REQUEST <= action < RECOG)
		out << "request to " << action - REQUEST << endl;
	else if (RECOG <= action < NO_ACTION) 
		out << "change recog state " << action - RECOG << endl;
	else
		out << "nothing" << endl;
}
				
