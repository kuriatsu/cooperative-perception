#include "ras.h"

#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot {


RasState::RasState() {
	ego_pose = 0;
	ego_speed = 11.2;
	vector<bool> ego_recog{Ras::NO_RISK, Ras::NO_RISK}; //TODO define based on the given situation
	req_time = 0;
	req_target = Ras::NO_ACTION;
    vector<int> risk_pose{80, 100};
	vector<bool> risk_bin{Ras::RISK, Ras::NO_RISK}; //TODO define based on the given situation
}

RasState::~RasState() {
}

string RasState::text() const {
	return "ego_pose: " + to_string(ego_pose) + "\n" + 
		   "ego_speed: " + to_string(ego_speed) + "\n" + 
		   "ego_recog: " + to_string(ego_recog) + "\n" +
		   "req_time: " + to_string(req_time) + "\n" +
		   "req_target: " + to_string(req_target) + "\n" +
		   "risk_bin: " + to_string(risk_bin) + "\n" +
           "weight:" + to_string(weight) + "\n";
}


int Ras::NumActions() const {
	return 1 + risks->size() * 2;
}

bool Ras::Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs)  const {
	RasState& state_curr = static_cast<RasState&>(state);
	RasState state_prev = state_curr;
	reward = 0.0;

	// ego state trantion
	EgoVehicleTransition(state_curr.ego_pose, state_curr.ego_speed, state_prev.ego_recog, risk_pose, action);

	// when action = change recog state 
	if (RECOG <= action && action < NO_ACTION) {
		int idx = action - RECOG;
		state_curr.ego_recog[idx] = !state_prev.ego_recog[idx];
		obs = NONE;
        state_curr.req_time = 1;
	}
	// when action = request intervention
	else if (REQUEST <= action && action < RECOG) {
		int idx = action - REQUEST;
		double acc = operator_model.int_acc(state_prev.req_time);
        state_curr.req_time += 1;

		// request to the same target
		if (state_prev.req_target == idx) {
			// observation probability
            if (rand_num < acc) {
                obs = (state_prev.risk_bin[idx] == RISK) ? RISK : NO_RISK;
            }
            else {
                obs = (state_prev.risk_bin[idx] == RISK) ? NO_RISK : RISK;
            }
		} 
		// request to new target
		else {
			state_curr.req_time = 1;
			state_curr.req_target = idx;
			obs = NONE;
		}
	}

	reward = CalcReward(state_prev, state_curr, risk_pose, action);

	if (state_curr.ego_pose >= planning_horizon)
		return true;
	else
		return false;
}


double Ras::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {

    if (REQUEST <= action && action < RECOG) {
        const RasState& ras_state = static_cast<const RasState&>(state);
        int idx = action - REQUEST;
        double acc = operator_model.int_acc(ras_state.req_time);
        // std::cout << "obs_prob acc : " << acc << " obs : " << obs << "action : " << action << "\n" << " state : " << ras_state << std::endl;
        return (ras_state.risk_bin[idx] == obs) ? acc : 1.0 - acc;
    }
    else {
        return 1.0;
    }
}


int Ras::CalcReward(const State& state_prev, const State& state_curr, const vector<int>& risk_poses, const ACT_TYPE& action) const {
	const RasState& _state_prev = static_cast<const RasState&>(state_prev);
	const RasState& _state_curr = static_cast<const RasState&>(state_curr);
	int reward = 0;

	for (auto it=risk_poses.begin(), end=risk_poses.end(); it != end; ++it) {
		int target_index = distance(risk_poses.begin(), it);
		if (_state_prev.ego_pose <= *it && *it < _state_curr.ego_pose) {

			// driving safety
			if (_state_curr.ego_recog[target_index] == _state_curr.risk_bin[target_index]) {
				reward += 1 * 1000;
                // std::cout << "conservative penal: " << reward << "pose: " << _state_prev.ego_pose << std::endl;
			}
            else if (_state_curr.ego_recog[target_index] == RISK && _state_curr.risk_bin[target_index] == NO_RISK) {
				reward += 1 * r_false_positive;
                // std::cout << "conservative penal: " << reward << "pose: " << _state_prev.ego_pose << std::endl;
			}
			else if (_state_curr.ego_recog[target_index] == NO_RISK && _state_curr.risk_bin[target_index] == RISK) {
				reward += 1 * r_false_negative;
                // std::cout << "aggressive penal: " << reward << "pose: " << _state_prev.ego_pose << "weigt: " << state_prev.weight << std::endl;
			}

            else {
                // driving efficiency
                // if (_state_curr.risk_bin[target_index] == NO_RISK) {
                    // when no risk, higher is better
                //     reward += (ideal_speed - _state_curr.ego_speed)/(ideal_speed - yield_speed) * r_eff;
                // }
                // else {
                    // when risk, lower is better
                //     reward += (_state_curr.ego_speed - yield_speed)/(ideal_speed - yield_speed) * r_eff;
                // }
            }
		}
	}

	// driving comfort (avoid harsh driving)
	reward += pow((_state_curr.ego_speed - _state_prev.ego_speed)/(ideal_speed - yield_speed), 2.0) * r_comf;
	
	// int request
	// if (REQUEST <= action && action < RECOG) {
	if (action != NO_ACTION) {
        reward += 1 * r_request;
	}

	return reward;
}


void Ras::EgoVehicleTransition(int& pose, double& speed, const vector<bool>& recog_list, const vector<int>& target_poses, const ACT_TYPE& action) const {
	vector<double> acc_list;

	if (speed < ideal_speed) {
		acc_list.emplace_back(ordinary_G);
	}
    else if (speed == ideal_speed) {
        acc_list.emplace_back(0.0);
	}
	else {
		acc_list.emplace_back(-ordinary_G);
	}

	for (auto it=recog_list.begin(), end=recog_list.end(); it != end; ++it) {
		int target_position = target_poses[distance(recog_list.begin(), it)];
        int dist = target_position - pose;
        bool is_decel_target = false; 

        if (dist < 0) {
            is_decel_target = false;
        }
        else if (REQUEST <= action && action < RECOG) {
            is_decel_target = (*it == RISK);
        }
        else {
            is_decel_target = (*it == RISK);
        }

        if (!is_decel_target){
            continue;
        }

        double a = 0.0;
        int decel_distance = (pow(speed, 2.0) - pow(yield_speed, 2))/(2.0*9.8*ordinary_G) + safety_margin;

        if (dist > decel_distance) {
            a = (pow(yield_speed, 2.0) - pow(speed, 2.0))/(2.0*(dist-safety_margin));
        }
        else {
            a = 0.0;
        }

        acc_list.emplace_back(a);
    }

    auto a_itr = min_element(acc_list.begin(), acc_list.end());
    double a = *a_itr;
    int &delta_t = Globals::config.time_per_move;
    // int decel_target = distance(acc_list.begin(), a_itr);
    speed += a * delta_t;
    if (speed <= yield_speed) {
        speed =	yield_speed;
        a = 0.0;
    }
    else if (speed >= ideal_speed) {
        speed = ideal_speed;
        a = 0.0;
    }

    pose += speed * delta_t + pow(0.5*a*delta_t, 2.0);
    return;
}


State* Ras::CreateStartState(string type) const {
	
	// set ego_recog and risk_bin based on threshold
    std::cout << "create state" << std::endl;
	vector<bool> _ego_recog, _risk_bin;
	for (auto val : risk_recog) {
        cout << "initial state risk_recog val " << val << "thesh " << risk_thresh << endl;
		_ego_recog.emplace_back((val < risk_thresh) ? NO_RISK : RISK);
		_risk_bin.emplace_back(RISK);
	}

	return new RasState(
			0, // ego_pose
			ideal_speed, // ego_speed
			_ego_recog, // ego_recog
			0, // req_time
			NO_ACTION, // req_target
			_risk_bin); // target_risk
}

Belief* Ras::InitialBelief(const State* start, string type) const {

	if (type != "DEFAULT" && type != "PARTICLE") {
		cout << "specified type " + type + " is not supported";
		exit(1);
	}

    // action index
    int risk_num = start->risk_bin.size();
    RECOG = risk_num+1;
    NO_ACTION = risk_num;

	// recognition likelihood of the automated system
    vector<bool> buf(risk_size, false);
	vector<vector<bool>> risk_bin_list;
	GetBinProduct(risk_bin_list, buf, 0); 
	vector<State*> particles;

	for (auto row : risk_bin_list) {
		double prob = 1.0;
		vector<bool> _ego_recog, _risk_bin;
		// set ego_recog and risk_bin based on threshold
		for (auto col=row.begin(), end=row.end(); col!=end; col++) {
			int idx = distance(row.begin(), col);
			_ego_recog.emplace_back((risk_recog[idx] < m_risk_thresh) ? NO_RISK : RISK);
			if (*col) {
				prob *= risk_recog[idx]; 
				_risk_bin.emplace_back(RISK);
			}
			else {
				prob *= 1.0 - risk_recog[idx]; 
				_risk_bin.emplace_back(NO_RISK);
			}
		}

        // TODO define based on the given sitiation
		RasState* p = static_cast<RasState*>(Allocate(-1, prob));  
		p->ego_pose = start->ego_pose;
		p->ego_speed = start->ego_speed;
		p->ego_recog = start->ego_recog;
		p->req_time = start->req_time;
	  	p->req_target = start->req_target;
		p->risk_bin = start->risk_bin;
        cout << *p << endl;
		particles.push_back(p);
	}
	return new ParticleBelief(particles, this);
}


// get every combination of the recognition state.
// [[true, true], [true, false], [false, true], [false, false]] for 2 obstacles
void Ras::GetBinProduct(vector<vector<bool>>& out_list, std::vector<bool> buf, int row) const {

    cout << "row" << row << "list" << out_list << endl;
    if (row == buf.size()) {
        out_list.emplace_back(buf);
        return;
    }

    for (int i=0; i<2; i++) {
        buf[row] = (i) ? false : true;   
        GetBinProduct(out_list, buf, row+1);
    }
}

double Ras::GetMaxReward() const {
	return 1000;
}

ValuedAction Ras::GetBestAction() const {
	return ValuedAction(NO_ACTION, 0);
}

State* Ras::Allocate(int state_id, double weight) const {
	RasState* ras_state = memory_pool.Allocate();
	ras_state->state_id = state_id;
	ras_state->weight = weight;
	return ras_state;
}

State* Ras::Copy(const State* particle) const {
	RasState* state = memory_pool.Allocate();
	*state = *static_cast<const RasState*>(particle);
	state->SetAllocated();
	return state;
}

void Ras::Free(State* particle) const {
	memory_pool.Free(static_cast<RasState*>(particle));
}

int Ras::NumActiveParticles() const {
	return memory_pool.num_allocated();
}

std::vector<double> Ras::getRiskProb(const Belief& belief) {
	const vector<State*>& particles = static_cast<const ParticleBelief&>(belief).particles();
	
	// double status = 0;
	vector<double> probs(risk_pose.size());
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		const RasState* state = static_cast<const RasState*>(particle);
		for (auto itr=state->risk_bin.begin(), end=state->risk_bin.end(); itr!=end; itr++) {
			probs[distance(state->risk_bin.begin(), itr)] += *itr * particle-> weight;
		}
	}
    return probs;
}

void Ras::printState(const State& state, ostream& out) const {
	const RasState& ras_state = static_cast<const RasState&>(state);
	out << "ego_pose : " << ras_state.ego_pose << "\n"
		<< "ego_speed : " << ras_state.ego_speed << "\n"
		<< "ego_recog : " << ras_state.ego_recog << "\n"
		<< "req_time : " << ras_state.req_time << "\n"
		<< "req_target : " << ras_state.req_target << "\n"
		<< "risk_bin : " << ras_state.risk_bin << "\n"
        << "weight : " << ras_state.weight << "\n"
		<< endl;
}

void Ras::PrintObs(const State& state, OBS_TYPE obs, ostream& out) const {
    switch(obs) {
        case NONE:
            out << "NONE" << endl;
            break;
        case NO_RISK:
            out << "NO_RISK" << endl;
            break;
        case RISK:
            out << "RISK" << endl;
            break;
    }
}

void Ras::PrintBelief(const Belief& belief, ostream& out) const {
	const vector<State*>& particles = static_cast<const ParticleBelief&>(belief).particles();
	
	// double status = 0;
	vector<double> probs(risk_pose.size());
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		const RasState* state = static_cast<const RasState*>(particle);
		for (auto itr=state->risk_bin.begin(), end=state->risk_bin.end(); itr!=end; itr++) {
			probs[distance(state->risk_bin.begin(), itr)] += *itr * particle-> weight;
		}
	}

	for (int i = 0; i < risk_pose.size(); i++) {
		out << "risk id : " << i << " prob : " << probs[i] << endl;
	}
}

void Ras::PrintAction(ACT_TYPE action, ostream& out) const {
	if (REQUEST <= action && action < RECOG)
		out << "request to " << action - REQUEST << endl;
	else if (RECOG <= action && action < NO_ACTION) 
		out << "change recog state " << action - RECOG << endl;
	else
		out << "nothing" << endl;
}
}
