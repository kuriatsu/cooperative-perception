#include "task_allocation.h"

#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot {


CPState::CPState() {
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

CPState::CPState(int _ego_pose, float _ego_speed, std::vector<bool> _ego_recog, int _req_time, int _req_target, std::vector<bool> _risk_bin, std::vector<int> _risk_pose) :
		ego_pose(_ego_pose),
		ego_speed(_ego_speed),
		ego_recog(_ego_recog),
		req_time(_req_time),
		req_target(_req_target),
		risk_pose(_risk_pose),
		risk_bin(_risk_bin)	{
}

CPState::~CPState() {
}

string CPState::text() const {
	return "ego_pose: " + to_string(ego_pose) + "\n" + 
	       "ego_speed: " + to_string(ego_speed) + "\n" + 
	 	   "ego_recog: " + to_string(ego_recog) + "\n" +
	 	   "req_time: " + to_string(req_time) + "\n" +
	 	   "req_target: " + to_string(req_target) + "\n" +
	 	   "risk_pose: " + to_string(risk_pose) + "\n" +
	 	   "risk_bin: " + to_string(risk_bin) + "\n"; 
    // return "";
}


class TADefaultPolicy: public DefaultPolicy {
protected:
    const CPPOMDP* task_allocation;
    const VehicleModel* vehicle_model;
    const OperatorModel* operator_model;
    const CPState* cp_values;
    
public:
    TADefaultPolicy(const CPPOMDP* model, ParticleLowerBound* bound) : 
        DefaultPolicy(model, bound),
        task_allocation(model), 
        vehicle_model(model->m_vehicle_model),
        operator_model(model->m_operator_model),
        cp_values(model->m_cp_values) {
        }

    ACT_TYPE Action(const vector<State*>& particles, RandomStreams& streams, History& history) const {
        const CPState& cp_state = static_cast<const CPState&>(*particles[0]);

        if (history.Size()) {
            ACT_TYPE action = history.LastAction();
            OBS_TYPE obs = history.LastObservation();

            // if (cp_values->getActionAttrib(action) == CPState::REQUEST) { 
            if (cp_state.req_time > 0) { 

                if (task_allocation->m_operator_model->int_acc(cp_state.req_time) <= 0.5) {
                    return cp_values->getAction(CPState::REQUEST, cp_state.req_target);
                }
                else if (task_allocation->m_operator_model->int_acc(cp_state.req_time) == 1.0) {
                    // if ((obs == CPState::RISK && cp_state.ego_recog[cp_state.req_target] == CPState::NO_RISK) || (obs == CPState::NO_RISK && cp_state.ego_recog[cp_state.req_target] == CPState::RISK)) {
                    if (obs != cp_state.ego_recog[cp_state.req_target]) {
                        // std::cout << "recog after request" << std::endl;
                        return cp_values->getAction(CPState::RECOG, cp_state.req_target);
                    }
                }
            }
            
//            else {
//                double comf_stop_dist = vehicle_model->getDecelDistance(cp_state.ego_speed, vehicle_model->m_min_decel, vehicle_model->m_safety_margin);
//                double harsh_stop_dist = vehicle_model->getDecelDistance(cp_state.ego_speed, vehicle_model->m_max_decel, 0.0);
//
//                // std::cout << "create list" << "comf_stop_dist : " << comf_stop_dist << " harsh_stop_dist : " << harsh_stop_dist << std::endl;
//                std::vector<int> recog_target_list, request_target_list;
//                for (int i=0; i<cp_state.risk_pose.size(); ++i) {
//
//                    // if (cp_state.ego_recog[i] == cp_state.risk_bin[i] || cp_state.risk_pose[i] - cp_state.ego_pose < harsh_stop_dist)
//                    if (cp_state.risk_pose[i] - cp_state.ego_pose < harsh_stop_dist)
//                        continue;
//
////                    else if (cp_state.risk_pose[i] - cp_state.ego_pose < comf_stop_dist) {
////                        bool in_history = false;
////                        for (int j = history.Size()-1; j >= 0; j--) {
////                            if (history.Action(j) == cp_values->getAction(CPState::RECOG, i)) {
////                                in_history = true;
////                                break;
////                            }
////                        }
////                        if (!in_history)
////                            recog_target_list.emplace_back(i);
////                    }
////                    else {
//
//
//
//                    if (cp_state.risk_pose[i] - cp_state.ego_pose >= comf_stop_dist) {
//                        bool in_history = false;
//                        for (int j = history.Size()-1; j >= 0; j--) {
//                            if (history.Action(j) == cp_values->getAction(CPState::REQUEST, i)) {
//                                // std::cout << "already in hist : " << i << std::endl;
//                                in_history = true;
//                                break;
//                            }
//                        }
//                        if (!in_history)
//                            request_target_list.emplace_back(i); 
//                    }
//                    if (cp_state.ego_recog[i] == CPState::NO_RISK) {
//                        recog_target_list.emplace_back(i);
//                    }
//                }
//
//                int recog_target = 1000, request_target = 1000, min_dist;
//                min_dist = 1000;
//                for(const auto idx : recog_target_list) {
//                    if (min_dist > cp_state.risk_pose[idx]) {
//                        min_dist = cp_state.risk_pose[idx];
//                        recog_target = idx;
//                    }
//                }
//                min_dist = 1000;
//                for(const auto idx : request_target_list) {
//                    if (min_dist > cp_state.risk_pose[idx]) {
//                        min_dist = cp_state.risk_pose[idx];
//                        request_target = idx;
//                    }
//                }

                // std::cout << "#########" << std::endl;
                // std::cout << cp_state.ego_pose << "," << harsh_stop_dist << "," << comf_stop_dist << cp_state.risk_bin << cp_state.ego_recog << request_target_list << recog_target_list << std::endl;
//                // std::cout << "compare" << cp_state.ego_pose << "," << request_target << "," << recog_target << "comf_stop_dist : " << comf_stop_dist << " harsh_stop_dist : " << harsh_stop_dist << std::endl;
                //    // std::cout << recog_target_list.size() << ", " << request_target_list.size() << std::endl;
//                if (recog_target_list.size()==0 && request_target_list.size()==0) { 
//                    // std::cout << "no action selected" << std::endl;
//                    return task_allocation->m_cp_values->getAction(CPState::NO_ACTION, 0);
//                }
//
//                else if (recog_target_list.empty()) {
//                    // std::cout <<  "request selected : " << request_target << std::endl;
//                    return cp_values->getAction(CPState::REQUEST, request_target);
//                }
//
//                else if (request_target_list.empty()) {
//                    // std::cout <<  "recog selected : " << recog_target << std::endl;
//                    return cp_values->getAction(CPState::RECOG, recog_target);
//                }
//                else if (cp_state.risk_pose[request_target] < cp_state.risk_pose[recog_target]) {
//                    // std::cout <<  "request selected : " << request_target << std::endl;
//                    return cp_values->getAction(CPState::REQUEST, request_target);
//                }
//                else {
//                    // std::cout <<  "recog selected : " << recog_target << std::endl;
//                    return cp_values->getAction(CPState::RECOG, recog_target);
//                }
//            }

        }
        return task_allocation->m_cp_values->getAction(CPState::NO_ACTION, 0);
    }
};


// class TAParticleUpperBound: public ParticleUpperBound {
// protected:
//     const CPPOMDP* task_allocation;
// public:
//     TAUpperBound(const CPPOMDP* model) : 
//         task_allocation(model) {
//     }
// 
//     double Value(const State& state) const {
//         const CPState& cp_state = static_cast<const CPState&>(state);
        

ScenarioUpperBound* CPPOMDP::CreateScenarioUpperBound(std::string name, std::string particle_bound_name) const {
    if (name == "TRIVIAL") {
        return new TrivialParticleUpperBound(this);
    }
    else if (name == "SMART" || name == "DEFAULT") {
        return new TrivialParticleUpperBound(this);
    }
    else {
        std::cerr << "Unsupported base upper bound: " << name << std::endl;
        exit(1);
        return NULL;
    }
}

ScenarioLowerBound* CPPOMDP::CreateScenarioLowerBound(std::string name, std::string particle_bound_name) const {

    if (name == "TRIVIAL") {
        return new TrivialParticleLowerBound(this);
    }
    else if (name == "SMART" || name == "DEFAULT") {
        return new TADefaultPolicy(this, CreateParticleLowerBound(particle_bound_name));
    }
    else {
        std::cerr << "Unsupported lower bound: " << name << std::endl;
        exit(0);
        return NULL;
    }
}


CPPOMDP::CPPOMDP(int planning_horizon, double risk_thresh, VehicleModel* vehicle_model, OperatorModel* operator_model, double delcp_t){ 
    m_planning_horizon = planning_horizon;
    m_risk_thresh = risk_thresh; 
    m_vehicle_model = vehicle_model;
    m_operator_model = operator_model;
    m_max_speed = m_vehicle_model->m_max_speed;
    m_yield_speed = m_vehicle_model->m_yield_speed;
    m_delcp_t = delcp_t;
}

CPPOMDP::CPPOMDP() {
    m_planning_horizon = 150;
    m_risk_thresh = 0.5; 
    // m_delcp_t = Globals::config.time_per_move;
    m_delcp_t = 2.0;

    m_vehicle_model = new VehicleModel();
    m_operator_model = new OperatorModel();
    m_start_state = new CPState();
    m_cp_values = new CPState(m_start_state->risk_pose.size());

    m_vehicle_model->m_delcp_t = m_delcp_t;
    m_max_speed = m_vehicle_model->m_max_speed;
    m_yield_speed = m_vehicle_model->m_yield_speed;

    m_recog_likelihood = {0.4, 0.4, 0.6};
}

CPPOMDP::CPPOMDP(const double delcp_t_, const std::vector<int> risk_pose_, const std::vector<double> risk_likelihood_) {
    m_planning_horizon = 150;
    m_risk_thresh = 0.5; 
    m_delcp_t = delcp_t_;

    m_vehicle_model = new VehicleModel();
    m_operator_model = new OperatorModel();

    m_recog_likelihood = risk_likelihood_;
    std::vector<bool> risk_bin;
    for (const auto likelihood : m_recog_likelihood) {
        risk_bin.emplace_back((likelihood > m_risk_thresh) ? true : false);
    }

    m_start_state = new CPState(0.0, m_max_speed, risk_bin, 0, 0, risk_bin, risk_pose_);
    m_cp_values = new CPState(m_start_state->risk_pose.size());

    m_vehicle_model->m_delcp_t = m_delcp_t;
    m_max_speed = m_vehicle_model->m_max_speed;
    m_yield_speed = m_vehicle_model->m_yield_speed;

}

int CPPOMDP::NumActions() const {
	return 1 + m_start_state->risk_pose.size() * 2;
}

bool CPPOMDP::Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs)  const {
	CPState& state_curr = static_cast<CPState&>(state);
	CPState state_prev = state_curr;
	reward = 0.0;

	// ego state trantion
	// EgoVehicleTransition(state_curr.ego_pose, state_curr.ego_speed, state_prev.ego_recog, risk_pose, action);
    m_vehicle_model->getTransition(state_curr.ego_speed, state_curr.ego_pose, state_prev.ego_recog, state_prev.risk_pose);
    
    int target_idx = m_cp_values->getActionTarget(action);
    CPState::ACT cp_action = m_cp_values->getActionAttrib(action);
    
    // when action == no_action
    if (cp_action == CPState::NO_ACTION) {
        // std::cout << "action : NO_ACTION" << std::endl;
        state_curr.req_time = 0;
        state_curr.req_target = 0;
        obs = m_operator_model->execIntervention(state_curr.req_time, cp_action, "", CPState::NO_RISK);
    }
    
	// when action = change recog state
    else if (cp_action == CPState::RECOG) {
        // std::cout << "action : RECOG" << std::endl;
		state_curr.ego_recog[target_idx] = (state_prev.ego_recog[target_idx] == CPState::RISK) ? CPState::NO_RISK : CPState::RISK;
        state_curr.req_time = 0;
        state_curr.req_target = 0;
        obs = m_operator_model->execIntervention(state_curr.req_time, cp_action, "", CPState::NO_RISK);
	}
    
	// when action = request intervention
	else if (cp_action == CPState::REQUEST) {
        // std::cout << "action : REQUEST" << std::endl;
        state_curr.ego_recog[target_idx] = CPState::RISK;

		// request to the same target
		if (state_prev.req_target == target_idx) {
            state_curr.req_time += m_delcp_t;
			state_curr.req_target = target_idx;
		} 
		// request to new target
		else {
			state_curr.req_time = m_delcp_t;
			state_curr.req_target = target_idx;
		}

        obs = m_operator_model->execIntervention(state_curr.req_time, cp_action, std::to_string(target_idx), state_curr.risk_bin[target_idx]);

	}
	reward = CalcReward(state_prev, state_curr, action);

	if (state_curr.ego_pose >= m_planning_horizon)
		return true;
	else
		return false;
}


double CPPOMDP::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {

    int target_idx = m_cp_values->getActionTarget(action);
    CPState::ACT cp_action = m_cp_values->getActionAttrib(action);

    if (cp_action == CPState::REQUEST) {
        const CPState& ras_state = static_cast<const CPState&>(state);
        double acc = m_operator_model->int_acc(ras_state.req_time);

        if (obs == CPState::NONE) return 0.0;
        return (ras_state.risk_bin[target_idx] == obs) ? acc : 1.0 - acc;
    }
    else {
        return obs == CPState::NONE;
        // return 1.0;
    }
}


int CPPOMDP::CalcReward(const State& _state_prev, const State& _state_curr, const ACT_TYPE& action) const {
	const CPState& state_prev = static_cast<const CPState&>(_state_prev);
	const CPState& state_curr = static_cast<const CPState&>(_state_curr);
	int reward = 0;

    int action_target_idx = m_cp_values->getActionTarget(action);
    CPState::ACT cp_action = m_cp_values->getActionAttrib(action);

	for (auto it=state_curr.risk_pose.begin(), end=state_curr.risk_pose.end(); it != end; ++it) {
		if (state_prev.ego_pose <= *it && *it < state_curr.ego_pose) {
            int passed_index = distance(state_curr.risk_pose.begin(), it);

			// driving safety
            reward += (state_prev.risk_bin[passed_index] == state_prev.ego_recog[passed_index]) ? 10 : -10;
            // if (state_prev.ego_recog[target_index] == CPState::NO_RISK)
            //    reward += (state_prev.risk_bin[target_index] == CPState::NO_RISK) ? 100 : -100;
            // else 
            //    reward += (state_prev.risk_bin[target_index] == CPState::RISK) ? 100 : -100;

// request intervention (same with recog==risk reward ?)
//            if (state_prev.ego_speed > m_yield_speed) {
//                reward += (state_prev.risk_bin[target_index] == CPState::RISK) ? -100 : 100;
//            }
//            else {
//                reward += (state_prev.risk_bin[target_index] == CPState::RISK) ? 100 : -100;
//            }

            // if (state_prev.ego_speed > m_yield_speed && state_curr.ego_speed > m_yield_speed) {
                // reward += (state_prev.risk_bin[passed_index] == CPState::RISK) ? -100 : 100;
            if (state_prev.risk_bin[passed_index] == CPState::RISK) {
                // reward += (m_max_speed - state_prev.ego_speed)/(m_max_speed - m_yield_speed) * 100;
                reward += (state_prev.ego_speed - m_yield_speed)/(m_max_speed - m_yield_speed) * -100;
                // reward += (m_max_speed - state_prev.ego_speed)/(m_max_speed - m_yield_speed) * 1000;
            }
            else {
                // reward += (m_max_speed - state_prev.ego_speed)/(m_max_speed - m_yield_speed) * -100;
                reward += (state_prev.ego_speed - m_yield_speed)/(m_max_speed - m_yield_speed) * 10;
            }
            
            // if (state_prev.ego_speed <= m_yield_speed) {
            //     reward += (state_prev.risk_bin[passed_index] == CPState::NO_RISK) ? -100 : 100;
            // }

            // if (state_prev.risk_bin[target_index] == CPState::RISK)
            //    reward += (state_prev.ego_speed <= m_yield_speed) ? 100 : -100;
            // else
            //     reward += (state_prev.ego_speed > m_yield_speed) ? 100 : -100;

            // try to keep mid speed
            // if (state_prev.ego_recog[target_index] == CPState::RISK)
            //     reward += (m_max_speed - state_prev.ego_speed)/(m_max_speed - m_yield_speed) * 10;
                // when risk, lower is better
            // else
                // when no risk, higher is better
            //    reward += (state_prev.ego_speed - m_yield_speed)/(m_max_speed - m_yield_speed) * 10;
		}
	}

	// driving comfort (avoid unnecessary speed change)
    // if (state_curr.ego_speed > state_prev.ego_speed) {
        // reward += -1;
        // reward += (state_curr.ego_speed - state_prev.ego_speed)/(m_max_speed - m_yield_speed) * -1;
    // }
	
    // driving efficiency
    // reward += -1;

    // if (cp_action == CPState::RECOG && state_prev.risk_bin[action_target_idx] != state_prev.ego_recog[action_target_idx])
    //     reward += -100;

	// int request
	// if (cp_action == CPState::REQUEST) {

    if (cp_action == CPState::RECOG) {
        // reward += -1;
        if (state_curr.risk_bin[action_target_idx] != state_curr.ego_recog[action_target_idx])
            reward += -1;
    }

	// if (cp_action == CPState::REQUEST) {
	if (cp_action == CPState::REQUEST && (state_curr.req_time == m_delcp_t || state_prev.req_target != state_curr.req_target)) {
        reward += 1 * -1 ;
	}

	return reward;
}

double CPPOMDP::GetMaxReward() const {
	return 1000;
}

ValuedAction CPPOMDP::GetBestAction() const {
	return ValuedAction(CPState::NO_ACTION, 0);
}


State* CPPOMDP::CreateStartState(string type) const {
    return m_start_state;
}

Belief* CPPOMDP::InitialBelief(const State* start, string type) const {
   
    const CPState *cp_start_state = static_cast<const CPState*>(start);
    m_cp_values = new CPState(cp_start_state->risk_pose.size());

	// recognition likelihood of the automated system
    vector<bool> buf(cp_start_state->risk_pose.size(), false);
	vector<vector<bool>> risk_bin_list;
	GetBinProduct(risk_bin_list, buf, 0); 
	vector<State*> particles;

	for (auto row : risk_bin_list) {
		double prob = 1.0;
		vector<bool> _ego_recog, _risk_bin;
		// set ego_recog and risk_bin based on threshold
		for (auto col=row.begin(), end=row.end(); col!=end; col++) {
			int idx = distance(row.begin(), col);
			_ego_recog.emplace_back((cp_start_state->ego_recog[idx] < m_risk_thresh) ? false : true);
			if (*col) {
				prob *= m_recog_likelihood[idx]; 
				// prob *= 0.5 ; 
				_risk_bin.emplace_back(true);
			}
			else {
				prob *= 1.0 - m_recog_likelihood[idx]; 
				// prob *= 0.5; 
				_risk_bin.emplace_back(false);
			}
		}

        // TODO define based on the given sitiation
		CPState* p = static_cast<CPState*>(Allocate(-1, prob));  
		p->ego_pose = cp_start_state->ego_pose;
		p->ego_speed = cp_start_state->ego_speed;
		p->ego_recog = cp_start_state->ego_recog;
		p->req_time = cp_start_state->req_time;
	  	p->req_target = cp_start_state->req_target;
	  	p->risk_pose = cp_start_state->risk_pose;
		p->risk_bin = _risk_bin;
        cout << *p << endl;
		particles.push_back(p);
	}
	return new ParticleBelief(particles, this);
}

Belief* CPPOMDP::InitialBelief(const State* start, const std::vector<double>& likelihood, std::string type) const {
   
    const CPState *cp_start_state = static_cast<const CPState*>(start);
    m_cp_values = new CPState(cp_start_state->risk_pose.size());

    if (likelihood.size() != cp_start_state->risk_pose.size()) {
        std::cout << "likelihood and risk have different list size!" << std::endl;
        exit(0);
    }

	// recognition likelihood of the automated system
    vector<bool> buf(cp_start_state->risk_pose.size(), false);
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
		CPState* p = static_cast<CPState*>(Allocate(-1, prob));  
		p->ego_pose = cp_start_state->ego_pose;
		p->ego_speed = cp_start_state->ego_speed;
		p->ego_recog = cp_start_state->ego_recog;
		p->req_time = cp_start_state->req_time;
	  	p->req_target = cp_start_state->req_target;
	  	p->risk_pose = cp_start_state->risk_pose;
		p->risk_bin = _risk_bin;
        cout << *p << endl;
		particles.push_back(p);
	}
    std::cout << "initial belief" << std::endl;
	return new ParticleBelief(particles, this);
}

// get every combination of the recognition state.
// [[true, true], [true, false], [false, true], [false, false]] for 2 obstacles
void CPPOMDP::GetBinProduct(vector<vector<bool>>& out_list, std::vector<bool> buf, int row) const {

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


State* CPPOMDP::Allocate(int state_id, double weight) const {
	CPState* ras_state = memory_pool.Allocate();
	ras_state->state_id = state_id;
	ras_state->weight = weight;
	return ras_state;
}

State* CPPOMDP::Copy(const State* particle) const {
	CPState* state = memory_pool.Allocate();
	*state = *static_cast<const CPState*>(particle);
	state->SetAllocated();
	return state;
}

void CPPOMDP::Free(State* particle) const {
	memory_pool.Free(static_cast<CPState*>(particle));
}

int CPPOMDP::NumActiveParticles() const {
	return memory_pool.num_allocated();
}

// void CPPOMDP::syncCurrentState(State* state, std::vector<double>& likelihood_list) {
//     std::cout << "sync current state" << std::endl;
//     m_start_state = static_cast<CPState*>(state);
    
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

//     m_cp_values = new CPState(m_start_state->risk_pose.size());
// }

std::vector<double> CPPOMDP::getRiskProb(const Belief* belief) {
	const vector<State*>& particles = static_cast<const ParticleBelief*>(belief)->particles();
	
	// double status = 0;
	vector<double> probs(m_start_state->risk_pose.size(), 0.0);
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		CPState* state = static_cast<CPState*>(particle);
		for (auto itr=state->risk_bin.begin(), end=state->risk_bin.end(); itr!=end; itr++) {
			probs[distance(state->risk_bin.begin(), itr)] += *itr * particle->weight;
		}
	}
    return probs;
}

void CPPOMDP::PrintState(const State& state, ostream& out) const {
	const CPState& ras_state = static_cast<const CPState&>(state);
	out << "ego_pose : " << ras_state.ego_pose << "\n"
 		<< "ego_speed : " << ras_state.ego_speed << "\n"
 		<< "ego_recog : " << ras_state.ego_recog << "\n"
 		<< "req_time : " << ras_state.req_time << "\n"
 		<< "risk_pose : " << ras_state.risk_pose << "\n"
 		<< "req_target : " << ras_state.req_target << "\n"
 		<< "risk_bin : " << ras_state.risk_bin << "\n"
        << "weight : " << ras_state.weight << "\n"
	 	<< endl;
}

void CPPOMDP::PrintObs(const State& state, OBS_TYPE obs, ostream& out) const {
    switch(obs) {
        case CPState::NONE:
            out << "NONE" << endl;
            break;
        case CPState::NO_RISK:
            out << "NO_RISK" << endl;
            break;
        case CPState::RISK:
            out << "RISK" << endl;
            break;
    }
}

void CPPOMDP::PrintBelief(const Belief& belief, ostream& out) const {
	const vector<State*>& particles = static_cast<const ParticleBelief&>(belief).particles();
	
	// double status = 0;
	vector<double> probs(m_start_state->risk_pose.size());
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
        const CPState* state = static_cast<const CPState*>(particle);
		for (auto itr=state->risk_bin.begin(), end=state->risk_bin.end(); itr!=end; itr++) {
			probs[distance(state->risk_bin.begin(), itr)] += *itr * particle-> weight;
		}
	}

	for (int i = 0; i < m_start_state->risk_pose.size(); i++) {
		out << "risk id : " << i << " prob : " << probs[i] << endl;
	}
}

void CPPOMDP::PrintAction(ACT_TYPE action, ostream& out) const {
    int target_idx = m_cp_values->getActionTarget(action);
    CPState::ACT cp_action = m_cp_values->getActionAttrib(action);

	if (cp_action == CPState::REQUEST)
		out << "request to " << target_idx << endl;
	else if (cp_action == CPState::RECOG) 
		out << "change recog state " << target_idx << endl;
	else
		out << "nothing" << endl;
}
}
