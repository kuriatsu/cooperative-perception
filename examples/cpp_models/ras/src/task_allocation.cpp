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
    ego_recog = {false, false, true};
    risk_pose = {60, 100, 120};
    risk_bin = {false, false, true};
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


class TADefaultPolicy: public DefaultPolicy {
protected:
    const TaskAllocation* task_allocation;
    const VehicleModel* vehicle_model;
    const OperatorModel* operator_model;
    const TAValues* ta_values;
    
public:
    TADefaultPolicy(const TaskAllocation* model, ParticleLowerBound* bound) : 
        DefaultPolicy(model, bound),
        task_allocation(model), 
        vehicle_model(model->m_vehicle_model),
        operator_model(model->m_operator_model),
        ta_values(model->m_ta_values) {
        }

    ACT_TYPE Action(const vector<State*>& particles, RandomStreams& streams, History& history) const {
        const TAState& ta_state = static_cast<const TAState&>(*particles[0]);

        if (history.Size()) {
            ACT_TYPE action = history.LastAction();
            OBS_TYPE obs = history.LastObservation();

            if (ta_values->getActionAttrib(action) == TAValues::REQUEST) { 

                if (task_allocation->m_operator_model->int_acc(ta_state.req_time) <= 0.5) {
                    return action;
                }
                else if (task_allocation->m_operator_model->int_acc(ta_state.req_time) == 1.0) {
                    // if ((obs == TAValues::RISK && ta_state.ego_recog[ta_state.req_target] == TAValues::NO_RISK) || (obs == TAValues::NO_RISK && ta_state.ego_recog[ta_state.req_target] == TAValues::RISK)) {
                    if (obs != ta_state.ego_recog[ta_state.req_target]) {
                        // std::cout << "recog after request" << std::endl;
                        return ta_values->getAction(TAValues::REQUEST, ta_state.req_target);
                    }
                }
            }
            
//            else {
//                double comf_stop_dist = vehicle_model->getDecelDistance(ta_state.ego_speed, vehicle_model->m_min_decel, vehicle_model->m_safety_margin);
//                double harsh_stop_dist = vehicle_model->getDecelDistance(ta_state.ego_speed, vehicle_model->m_max_decel, 0.0);
//
//                // std::cout << "create list" << "comf_stop_dist : " << comf_stop_dist << " harsh_stop_dist : " << harsh_stop_dist << std::endl;
//                std::vector<int> recog_target_list, request_target_list;
//                for (int i=0; i<ta_state.risk_pose.size(); ++i) {
//
//                    // if (ta_state.ego_recog[i] == ta_state.risk_bin[i] || ta_state.risk_pose[i] - ta_state.ego_pose < harsh_stop_dist)
//                    if (ta_state.risk_pose[i] - ta_state.ego_pose < harsh_stop_dist)
//                        continue;
//
////                    else if (ta_state.risk_pose[i] - ta_state.ego_pose < comf_stop_dist) {
////                        bool in_history = false;
////                        for (int j = history.Size()-1; j >= 0; j--) {
////                            if (history.Action(j) == ta_values->getAction(TAValues::RECOG, i)) {
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
//                    if (ta_state.risk_pose[i] - ta_state.ego_pose >= comf_stop_dist) {
//                        bool in_history = false;
//                        for (int j = history.Size()-1; j >= 0; j--) {
//                            if (history.Action(j) == ta_values->getAction(TAValues::REQUEST, i)) {
//                                // std::cout << "already in hist : " << i << std::endl;
//                                in_history = true;
//                                break;
//                            }
//                        }
//                        if (!in_history)
//                            request_target_list.emplace_back(i); 
//                    }
//                    if (ta_state.ego_recog[i] == TAValues::NO_RISK) {
//                        recog_target_list.emplace_back(i);
//                    }
//                }
//
//                int recog_target = 1000, request_target = 1000, min_dist;
//                min_dist = 1000;
//                for(const auto idx : recog_target_list) {
//                    if (min_dist > ta_state.risk_pose[idx]) {
//                        min_dist = ta_state.risk_pose[idx];
//                        recog_target = idx;
//                    }
//                }
//                min_dist = 1000;
//                for(const auto idx : request_target_list) {
//                    if (min_dist > ta_state.risk_pose[idx]) {
//                        min_dist = ta_state.risk_pose[idx];
//                        request_target = idx;
//                    }
//                }

                // std::cout << "#########" << std::endl;
                // std::cout << ta_state.ego_pose << "," << harsh_stop_dist << "," << comf_stop_dist << ta_state.risk_bin << ta_state.ego_recog << request_target_list << recog_target_list << std::endl;
//                // std::cout << "compare" << ta_state.ego_pose << "," << request_target << "," << recog_target << "comf_stop_dist : " << comf_stop_dist << " harsh_stop_dist : " << harsh_stop_dist << std::endl;
                //    // std::cout << recog_target_list.size() << ", " << request_target_list.size() << std::endl;
//                if (recog_target_list.size()==0 && request_target_list.size()==0) { 
//                    // std::cout << "no action selected" << std::endl;
//                    return task_allocation->m_ta_values->getAction(TAValues::NO_ACTION, 0);
//                }
//
//                else if (recog_target_list.empty()) {
//                    // std::cout <<  "request selected : " << request_target << std::endl;
//                    return ta_values->getAction(TAValues::REQUEST, request_target);
//                }
//
//                else if (request_target_list.empty()) {
//                    // std::cout <<  "recog selected : " << recog_target << std::endl;
//                    return ta_values->getAction(TAValues::RECOG, recog_target);
//                }
//                else if (ta_state.risk_pose[request_target] < ta_state.risk_pose[recog_target]) {
//                    // std::cout <<  "request selected : " << request_target << std::endl;
//                    return ta_values->getAction(TAValues::REQUEST, request_target);
//                }
//                else {
//                    // std::cout <<  "recog selected : " << recog_target << std::endl;
//                    return ta_values->getAction(TAValues::RECOG, recog_target);
//                }
//            }

        }
        return task_allocation->m_ta_values->getAction(TAValues::NO_ACTION, 0);
    }
};


// class TAParticleUpperBound: public ParticleUpperBound {
// protected:
//     const TaskAllocation* task_allocation;
// public:
//     TAUpperBound(const TaskAllocation* model) : 
//         task_allocation(model) {
//     }
// 
//     double Value(const State& state) const {
//         const TAState& ta_state = static_cast<const TAState&>(state);
        

ScenarioUpperBound* TaskAllocation::CreateScenarioUpperBound(std::string name, std::string particle_bound_name) const {
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

ScenarioLowerBound* TaskAllocation::CreateScenarioLowerBound(std::string name, std::string particle_bound_name) const {

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
    m_delta_t = 2.0;

    m_vehicle_model = new VehicleModel();
    m_operator_model = new OperatorModel();
    m_start_state = new TAState();
    m_ta_values = new TAValues(m_start_state->risk_pose.size());

    m_vehicle_model->m_delta_t = m_delta_t;
    m_max_speed = m_vehicle_model->m_max_speed;
    m_yield_speed = m_vehicle_model->m_yield_speed;

    m_recog_likelihood = {0.4, 0.4, 0.6};
}

TaskAllocation::TaskAllocation(const double delta_t_, const std::vector<int> risk_pose_, const std::vector<double> risk_likelihood_) {
    m_planning_horizon = 150;
    m_risk_thresh = 0.5; 
    m_delta_t = delta_t_;

    m_vehicle_model = new VehicleModel();
    m_operator_model = new OperatorModel();

    m_recog_likelihood = risk_likelihood_;
    std::vector<bool> risk_bin;
    for (const auto likelihood : m_recog_likelihood) {
        risk_bin.emplace_back((likelihood > m_risk_thresh) ? true : false);
    }

    m_start_state = new TAState(0.0, m_max_speed, risk_bin, 0, 0, risk_bin, risk_pose_);
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
    
    int target_idx = m_ta_values->getActionTarget(action);
    TAValues::ACT ta_action = m_ta_values->getActionAttrib(action);
    
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

    int target_idx = m_ta_values->getActionTarget(action);
    TAValues::ACT ta_action = m_ta_values->getActionAttrib(action);

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

    int action_target_idx = m_ta_values->getActionTarget(action);
    TAValues::ACT ta_action = m_ta_values->getActionAttrib(action);

	for (auto it=state_curr.risk_pose.begin(), end=state_curr.risk_pose.end(); it != end; ++it) {
		if (state_prev.ego_pose <= *it && *it < state_curr.ego_pose) {
            int passed_index = distance(state_curr.risk_pose.begin(), it);

			// driving safety
            reward += (state_prev.risk_bin[passed_index] == state_prev.ego_recog[passed_index]) ? 10 : -10;
            // if (state_prev.ego_recog[target_index] == TAValues::NO_RISK)
            //    reward += (state_prev.risk_bin[target_index] == TAValues::NO_RISK) ? 100 : -100;
            // else 
            //    reward += (state_prev.risk_bin[target_index] == TAValues::RISK) ? 100 : -100;

// request intervention (same with recog==risk reward ?)
//            if (state_prev.ego_speed > m_yield_speed) {
//                reward += (state_prev.risk_bin[target_index] == TAValues::RISK) ? -100 : 100;
//            }
//            else {
//                reward += (state_prev.risk_bin[target_index] == TAValues::RISK) ? 100 : -100;
//            }

            // if (state_prev.ego_speed > m_yield_speed && state_curr.ego_speed > m_yield_speed) {
                // reward += (state_prev.risk_bin[passed_index] == TAValues::RISK) ? -100 : 100;
            if (state_prev.risk_bin[passed_index] == TAValues::RISK) {
                // reward += (m_max_speed - state_prev.ego_speed)/(m_max_speed - m_yield_speed) * 100;
                reward += (state_prev.ego_speed - m_yield_speed)/(m_max_speed - m_yield_speed) * -100;
                // reward += (m_max_speed - state_prev.ego_speed)/(m_max_speed - m_yield_speed) * 1000;
            }
            else {
                // reward += (m_max_speed - state_prev.ego_speed)/(m_max_speed - m_yield_speed) * -100;
                reward += (state_prev.ego_speed - m_yield_speed)/(m_max_speed - m_yield_speed) * 10;
            }
            // }
            // if (state_prev.ego_speed <= m_yield_speed) {
            //     reward += (state_prev.risk_bin[passed_index] == TAValues::NO_RISK) ? -100 : 100;
            // }

            // if (state_prev.risk_bin[target_index] == TAValues::RISK)
            //    reward += (state_prev.ego_speed <= m_yield_speed) ? 100 : -100;
            // else
            //     reward += (state_prev.ego_speed > m_yield_speed) ? 100 : -100;

            // try to keep mid speed
            // if (state_prev.ego_recog[target_index] == TAValues::RISK)
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

    // if (ta_action == TAValues::RECOG && state_prev.risk_bin[action_target_idx] != state_prev.ego_recog[action_target_idx])
    //     reward += -100;

	// int request
	// if (ta_action == TAValues::REQUEST) {

    // if (ta_action == TAValues::RECOG) {
    //     if (state_curr.risk_bin[action_target_idx] != state_curr.ego_recog[action_target_idx])
    //         reward += -1;
    // }

	if (ta_action == TAValues::REQUEST && state_curr.req_time == m_delta_t) {
        reward += 1 * -1 ;
	}

	return reward;
}

double TaskAllocation::GetMaxReward() const {
	return 1000;
}

ValuedAction TaskAllocation::GetBestAction() const {
	return ValuedAction(TAValues::NO_ACTION, 0);
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
    std::cout << "sync current state" << std::endl;
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
    int target_idx = m_ta_values->getActionTarget(action);
    TAValues::ACT ta_action = m_ta_values->getActionAttrib(action);

	if (ta_action == TAValues::REQUEST)
		out << "request to " << target_idx << endl;
	else if (ta_action == TAValues::RECOG) 
		out << "change recog state " << target_idx << endl;
	else
		out << "nothing" << endl;
}
}
