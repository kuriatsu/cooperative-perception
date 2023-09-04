#include "task_allocation.h"

#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot {


TAState::TAState() {
    ego_pose = 0;
    ego_speed = 11.2;
    req_time = 0;
    req_target = 0;
    // ego_recog = {false, true, true};
    // risk_pose = {80, 100, 120};
    // risk_bin = {true, true, false};
    ego_recog = {false, true, false};
    risk_pose = {60, 120, 140};
    risk_bin = {true, false, true};
}


TAState::TAState(int _ego_pose, float _ego_speed, std::vector<bool> _ego_recog, int _req_time, int _req_target, std::vector<bool> _risk_bin, std::vector<int> _risk_pose) :
		ego_pose(_ego_pose),
		ego_speed(_ego_speed),
		ego_recog(_ego_recog),
		req_time(_req_time),
		req_target(_req_target),
		risk_pose(_risk_pose),
		risk_bin(_risk_bin)	{
}

TAState::~TAState() {
}

string TAState::text() const {
	return "ego_pose: " + to_string(ego_pose) + "\n" + 
	       "ego_speed: " + to_string(ego_speed) + "\n" + 
	 	   "ego_recog: " + to_string(ego_recog) + "\n" +
	 	   "req_time: " + to_string(req_time) + "\n" +
	 	   "req_target: " + to_string(req_target) + "\n" +
	 	   "risk_pose: " + to_string(risk_pose) + "\n" +
	 	   "risk_bin: " + to_string(risk_bin) + "\n"; 
    // return "";
}

TaskAllocation::TaskAllocation(int planning_horizon, double risk_thresh, VehicleModel* vehicle_model, OperatorModel* operator_model, double delta_t){ 
    m_planning_horizon = planning_horizon;
    m_risk_thresh = risk_thresh; 
    m_vehicle_model = vehicle_model;
    m_operator_model = operator_model;
    m_max_speed = m_vehicle_model->m_max_speed;
    m_yield_speed = m_vehicle_model->m_yield_speed;
    m_delta_t = delta_t;
}

TaskAllocation::TaskAllocation() {
    m_planning_horizon = 150;
    m_risk_thresh = 0.5; 
    // m_delta_t = Globals::config.time_per_move;
    m_delta_t = 1.0;

    m_vehicle_model = new VehicleModel();
    m_operator_model = new OperatorModel();
    m_start_state = new TAState();
    m_ta_values = new TAValues(m_start_state->risk_pose.size());

    m_vehicle_model->m_delta_t = m_delta_t;
    m_max_speed = m_vehicle_model->m_max_speed;
    m_yield_speed = m_vehicle_model->m_yield_speed;
}

int TaskAllocation::NumActions() const {
	return 1 + m_start_state->risk_pose.size() * 2;
}

bool TaskAllocation::Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs)  const {
	TAState& state_curr = static_cast<TAState&>(state);
	TAState state_prev = state_curr;
	reward = 0.0;

	// ego state trantion
	// EgoVehicleTransition(state_curr.ego_pose, state_curr.ego_speed, state_prev.ego_recog, risk_pose, action);
    m_vehicle_model->getTransition(state_curr.ego_speed, state_curr.ego_pose, state_prev.ego_recog, state_prev.risk_pose);
    
    int target_idx;
    TAValues::ACT ta_action = m_ta_values->getActionTarget(action, target_idx);
    
    // when action == no_action
    if (ta_action == TAValues::NO_ACTION) {
        // std::cout << "action : NO_ACTION" << std::endl;
        state_curr.req_time = 0;
        state_curr.req_target = 0;
        obs = m_operator_model->execIntervention(state_curr.req_time, ta_action, "", TAValues::NO_RISK);
    }
    
	// when action = change recog state
    else if (ta_action == TAValues::RECOG) {
        // std::cout << "action : RECOG" << std::endl;
		state_curr.ego_recog[target_idx] = (state_prev.ego_recog[target_idx] == TAValues::RISK) ? TAValues::NO_RISK : TAValues::RISK;
        state_curr.req_time = 0;
        state_curr.req_target = 0;
        obs = m_operator_model->execIntervention(state_curr.req_time, ta_action, "", TAValues::NO_RISK);
	}
    
	// when action = request intervention
	else if (ta_action == TAValues::REQUEST) {
        // std::cout << "action : REQUEST" << std::endl;
        state_curr.ego_recog[target_idx] = TAValues::RISK;

		// request to the same target
		if (state_prev.req_target == target_idx) {
            state_curr.req_time += m_delta_t;
			state_curr.req_target = target_idx;
		} 
		// request to new target
		else {
			state_curr.req_time = m_delta_t;
			state_curr.req_target = target_idx;
		}

        obs = m_operator_model->execIntervention(state_curr.req_time, ta_action, std::to_string(target_idx), state_curr.risk_bin[target_idx]);

	}
	reward = CalcReward(state_prev, state_curr, action);

	if (state_curr.ego_pose >= m_planning_horizon)
		return true;
	else
		return false;
}


double TaskAllocation::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {

    int target_idx;
    TAValues::ACT ta_action = m_ta_values->getActionTarget(action, target_idx);
    if (ta_action == TAValues::REQUEST) {
        const TAState& ras_state = static_cast<const TAState&>(state);
        double acc = m_operator_model->int_acc(ras_state.req_time);

        if (obs == TAValues::NONE) return 0.0;
        return (ras_state.risk_bin[target_idx] == obs) ? acc : 1.0 - acc;
    }
    else {
        return obs == TAValues::NONE;
        // return 1.0;
    }
}


int TaskAllocation::CalcReward(const State& _state_prev, const State& _state_curr, const ACT_TYPE& action) const {
	const TAState& state_prev = static_cast<const TAState&>(_state_prev);
	const TAState& state_curr = static_cast<const TAState&>(_state_curr);
	int reward = 0;

    int target_idx;
    TAValues::ACT ta_action = m_ta_values->getActionTarget(action, target_idx);

	for (auto it=state_curr.risk_pose.begin(), end=state_curr.risk_pose.end(); it != end; ++it) {
		int target_index = distance(state_curr.risk_pose.begin(), it);
		if (state_prev.ego_pose <= *it && *it < state_curr.ego_pose) {

			// driving safety
//             reward += (state_prev.ego_recog[target_index] == state_prev.risk_bin[target_index]) ? 100 : -100;

// request intervention (same with recog==risk reward ?)
            if (state_prev.risk_bin[target_index] == TAValues::RISK)
                reward += (state_prev.ego_speed <= m_yield_speed) ? 100 : -100;
//            else
//                reward += (state_prev.ego_speed > m_yield_speed) ? 100 : -100;

// try to keep mid speed
            if (state_prev.risk_bin[target_index] == TAValues::NO_RISK)
                // when risk, lower is better
                reward += (state_prev.ego_speed - m_yield_speed)/(m_max_speed - m_yield_speed) * 100;
//            else
//                // when no risk, higher is better
//                reward += (m_max_speed - state_prev.ego_speed)/(m_max_speed - m_yield_speed) * 10;
		}
	}

	// driving comfort (avoid unnecessary speed change)
    // if (state_curr.ego_speed > state_prev.ego_speed) {
        // reward += -1;
        // reward += (state_curr.ego_speed - state_prev.ego_speed)/(m_max_speed - m_yield_speed) * -1;
    // }
	
    // driving efficiency
    // reward += -1;

	// int request
	if (ta_action == TAValues::REQUEST && state_curr.req_time == m_delta_t) {
        reward += 1 * -1 ;
	}

	return reward;
}

double TaskAllocation::GetMaxReward() const {
	return 1000;
}

ValuedAction TaskAllocation::GetBestAction() const {
	return ValuedAction(TAValues::NO_ACTION, -1);
}


State* TaskAllocation::CreateStartState(string type) const {
    return m_start_state;
}

Belief* TaskAllocation::InitialBelief(const State* start, string type) const {
   
    const TAState *ta_start_state = static_cast<const TAState*>(start);

	// recognition likelihood of the automated system
    vector<bool> buf(ta_start_state->risk_pose.size(), false);
	vector<vector<bool>> risk_bin_list;
	GetBinProduct(risk_bin_list, buf, 0); 
	vector<State*> particles;

	for (auto row : risk_bin_list) {
		double prob = 1.0;
		vector<bool> _ego_recog, _risk_bin;
		// set ego_recog and risk_bin based on threshold
		for (auto col=row.begin(), end=row.end(); col!=end; col++) {
			int idx = distance(row.begin(), col);
			_ego_recog.emplace_back((ta_start_state->ego_recog[idx] < m_risk_thresh) ? false : true);
			if (*col) {
				prob *= 0.5 ; 
				_risk_bin.emplace_back(true);
			}
			else {
				prob *= 0.5; 
				_risk_bin.emplace_back(false);
			}
		}

        // TODO define based on the given sitiation
		TAState* p = static_cast<TAState*>(Allocate(-1, prob));  
		p->ego_pose = ta_start_state->ego_pose;
		p->ego_speed = ta_start_state->ego_speed;
		p->ego_recog = ta_start_state->ego_recog;
		p->req_time = ta_start_state->req_time;
	  	p->req_target = ta_start_state->req_target;
	  	p->risk_pose = ta_start_state->risk_pose;
		p->risk_bin = _risk_bin;
        cout << *p << endl;
		particles.push_back(p);
	}
	return new ParticleBelief(particles, this);
}

Belief* TaskAllocation::InitialBelief(const State* start, const std::vector<double>& likelihood, std::string type) const {
   
    const TAState *ta_start_state = static_cast<const TAState*>(start);

    if (likelihood.size() != ta_start_state->risk_pose.size()) {
        std::cout << "likelihood and risk have different list size!" << std::endl;
        exit(0);
    }

	// recognition likelihood of the automated system
    vector<bool> buf(ta_start_state->risk_pose.size(), false);
	vector<vector<bool>> risk_bin_list;
	GetBinProduct(risk_bin_list, buf, 0); 
	vector<State*> particles;

	for (auto row : risk_bin_list) {
		double prob = 1.0;
		vector<bool> _risk_bin;
		// set ego_recog and risk_bin based on threshold
		for (auto col=row.begin(), end=row.end(); col!=end; col++) {
			int idx = distance(row.begin(), col);
			if (*col) {
				prob *= likelihood[idx]; 
				_risk_bin.emplace_back(true);
			}
			else {
				prob *= 1.0 - likelihood[idx]; 
				_risk_bin.emplace_back(false);
			}
		}

        // TODO define based on the given sitiation
		TAState* p = static_cast<TAState*>(Allocate(-1, prob));  
		p->ego_pose = ta_start_state->ego_pose;
		p->ego_speed = ta_start_state->ego_speed;
		p->ego_recog = ta_start_state->ego_recog;
		p->req_time = ta_start_state->req_time;
	  	p->req_target = ta_start_state->req_target;
	  	p->risk_pose = ta_start_state->risk_pose;
		p->risk_bin = _risk_bin;
        cout << *p << endl;
		particles.push_back(p);
	}
    std::cout << "initial belief" << std::endl;
	return new ParticleBelief(particles, this);
}

// get every combination of the recognition state.
// [[true, true], [true, false], [false, true], [false, false]] for 2 obstacles
void TaskAllocation::GetBinProduct(vector<vector<bool>>& out_list, std::vector<bool> buf, int row) const {

    // check value combination
    // cout << "row" << row << "list" << out_list << endl;
    if (row == buf.size()) {
        out_list.emplace_back(buf);
        return;
    }

    for (int i=0; i<2; i++) {
        buf[row] = (i) ? false : true;   
        GetBinProduct(out_list, buf, row+1);
    }
}


State* TaskAllocation::Allocate(int state_id, double weight) const {
	TAState* ras_state = memory_pool.Allocate();
	ras_state->state_id = state_id;
	ras_state->weight = weight;
	return ras_state;
}

State* TaskAllocation::Copy(const State* particle) const {
	TAState* state = memory_pool.Allocate();
	*state = *static_cast<const TAState*>(particle);
	state->SetAllocated();
	return state;
}

void TaskAllocation::Free(State* particle) const {
	memory_pool.Free(static_cast<TAState*>(particle));
}

int TaskAllocation::NumActiveParticles() const {
	return memory_pool.num_allocated();
}

void TaskAllocation::syncCurrentState(State* state, std::vector<double>& likelihood_list) {
    m_start_state = static_cast<TAState*>(state);
    
//    // target number up to 3
//    if (m_start_state->risk_pose.size() > m_max_perception_num) {
//        std::vector<double> pose_list;
//        for (const auto pose : m_start_state->risk_pose) {
//            pose_list.emplace_back(pose);
//        }
//        std::sort(pose_list.begin(), pose_list.end());
//        
//        // remove targets farther than the 4th target
//        m_planning_horizon = pose_list[m_max_perception_num];
//        for (auto i=0; i<m_start_state->risk_pose.size();) {
//            if (m_start_state->risk_pose[i] >= m_planning_horizon) {
//                m_start_state->risk_pose.erase(m_start_state->risk_pose.begin() + i);
//                m_start_state->risk_bin.erase(m_start_state->risk_bin.begin() + i);
//                m_start_state->ego_recog.erase(m_start_state->ego_recog.begin() + i);
//                likelihood_list.erase(likelihood_list.begin() + i);
//            }
//            else {
//                ++i;
//            }
//        }
//    }
//    else {
//        m_planning_horizon = 150;
//    }

    m_ta_values = new TAValues(m_start_state->risk_pose.size());
}

std::vector<double> TaskAllocation::getRiskProb(const Belief* belief) {
	const vector<State*>& particles = static_cast<const ParticleBelief*>(belief)->particles();
	
	// double status = 0;
	vector<double> probs(m_start_state->risk_pose.size(), 0.0);
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		TAState* state = static_cast<TAState*>(particle);
		for (auto itr=state->risk_bin.begin(), end=state->risk_bin.end(); itr!=end; itr++) {
			probs[distance(state->risk_bin.begin(), itr)] += *itr * particle-> weight;
		}
	}
    return probs;
}

void TaskAllocation::PrintState(const State& state, ostream& out) const {
	const TAState& ras_state = static_cast<const TAState&>(state);
	out << "ego_pose : " << ras_state.ego_pose << "\n"
 		<< "ego_speed : " << ras_state.ego_speed << "\n"
 		<< "ego_recog : " << ras_state.ego_recog << "\n"
 		<< "req_time : " << ras_state.req_time << "\n"
 		<< "req_target : " << ras_state.req_target << "\n"
 		<< "risk_bin : " << ras_state.risk_bin << "\n"
        << "weight : " << ras_state.weight << "\n"
	 	<< endl;
}

void TaskAllocation::PrintObs(const State& state, OBS_TYPE obs, ostream& out) const {
    switch(obs) {
        case TAValues::NONE:
            out << "NONE" << endl;
            break;
        case TAValues::NO_RISK:
            out << "NO_RISK" << endl;
            break;
        case TAValues::RISK:
            out << "RISK" << endl;
            break;
    }
}

void TaskAllocation::PrintBelief(const Belief& belief, ostream& out) const {
	const vector<State*>& particles = static_cast<const ParticleBelief&>(belief).particles();
	
	// double status = 0;
	vector<double> probs(m_start_state->risk_pose.size());
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
        const TAState* state = static_cast<const TAState*>(particle);
		for (auto itr=state->risk_bin.begin(), end=state->risk_bin.end(); itr!=end; itr++) {
			probs[distance(state->risk_bin.begin(), itr)] += *itr * particle-> weight;
		}
	}

	for (int i = 0; i < m_start_state->risk_pose.size(); i++) {
		out << "risk id : " << i << " prob : " << probs[i] << endl;
	}
}

void TaskAllocation::PrintAction(ACT_TYPE action, ostream& out) const {
    int target_idx;
    TAValues::ACT ta_action = m_ta_values->getActionTarget(action, target_idx);
	if (ta_action == TAValues::REQUEST)
		out << "request to " << target_idx << endl;
	else if (ta_action == TAValues::RECOG) 
		out << "change recog state " << target_idx << endl;
	else
		out << "nothing" << endl;
}
}
