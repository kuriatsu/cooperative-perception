#include "task_allocation.h"

#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot {


TAState::TAState() {
    ego_pose = 0;
    ego_speed = 0;
    req_time = 0;
    req_target = 0;
    ego_recog = {false, false, true};
    risk_pose = {60, 100, 120};
    risk_bin = {false, false, true};
    risk_type = {"easy", "easy", "hard"};
}

TAState::TAState(int _ego_pose, float _ego_speed, std::vector<bool> _ego_recog, int _req_time, int _req_target, std::vector<bool> _risk_bin, std::vector<int> _risk_pose, std::vector<std::string> _risk_type) :
		ego_pose(_ego_pose),
		ego_speed(_ego_speed),
		ego_recog(_ego_recog),
		req_time(_req_time),
		req_target(_req_target),
		risk_pose(_risk_pose),
		risk_bin(_risk_bin),
		risk_type(_risk_type)	{
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
	 	   "risk_bin: " + to_string(risk_bin) + "\n"+ 
	 	   "risk_type: " + to_string(risk_type) + "\n"; 
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
        vehicle_model(model->_vehicle_model),
        operator_model(model->_operator_model),
        ta_values(model->_ta_values) {
        }

    ACT_TYPE Action(const vector<State*>& particles, RandomStreams& streams, History& history) const {
        const TAState& ta_state = static_cast<const TAState&>(*particles[0]);

        if (history.Size()) {
            ACT_TYPE action = history.LastAction();
            OBS_TYPE obs = history.LastObservation();

            // if (ta_values->getActionAttrib(action) == TAValues::REQUEST) { 
            if (ta_state.req_time > 0) { 

                if (task_allocation->_operator_model->intAcc(ta_state.req_time, ta_state.risk_type[ta_state.req_target]) <= 0.5) {
                    return ta_values->getAction(TAValues::REQUEST, ta_state.req_target);
                }
            }
        }
        return task_allocation->_ta_values->getAction(TAValues::NO_ACTION, 0);
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
    _planning_horizon = planning_horizon;
    _risk_thresh = risk_thresh; 
    _vehicle_model = vehicle_model;
    _operator_model = operator_model;
    _delta_t = delta_t;
}

TaskAllocation::TaskAllocation() :
    _planning_horizon(150),
    _risk_thresh(0.5),
    _delta_t(1.0) {

    _vehicle_model = new VehicleModel();
    _operator_model = new OperatorModel();
    _start_state = new TAState();
    _ta_values = new TAValues(_start_state->risk_pose.size());
    _vehicle_model->_delta_t = _delta_t;
    _recog_likelihood = {0.4, 0.4, 0.6};
}

TaskAllocation::TaskAllocation(const double delta_t_, TAState* start_state, std::vector<double> risk_likelihood_) :
    _planning_horizon(150),
    _risk_thresh(0.5),
    _delta_t(delta_t_) {

    _vehicle_model = new VehicleModel(_delta_t);
    _operator_model = new OperatorModel();

    _recog_likelihood = risk_likelihood_;
    std::vector<bool> risk_bin;
    for (const auto likelihood : _recog_likelihood) {
        risk_bin.emplace_back((likelihood >= _risk_thresh) ? true : false);
    }

    _start_state = start_state;
    _ta_values = new TAValues(_start_state->risk_pose.size());

    _recog_likelihood = risk_likelihood_;

}

int TaskAllocation::NumActions() const {
	return 1 + _start_state->risk_pose.size();
}

bool TaskAllocation::Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs)  const {
	TAState& state_curr = static_cast<TAState&>(state);
	TAState state_prev = state_curr;
	reward = 0.0;

	// ego state trantion
	// EgoVehicleTransition(state_curr.ego_pose, state_curr.ego_speed, state_prev.ego_recog, risk_pose, action);
    _vehicle_model->getTransition(state_curr.ego_speed, state_curr.ego_pose, state_prev.ego_recog, state_prev.risk_pose);
 
    int target_idx = _ta_values->getActionTarget(action);
    TAValues::ACT ta_action = _ta_values->getActionAttrib(action);
    
    // when action == no_action
    if (ta_action == TAValues::NO_ACTION) {
        // std::cout << "action : NO_ACTION" << std::endl;
        state_curr.req_time = 0;
        obs = _operator_model->execIntervention(0, false, rand_num, "");
    }
    
	// when action = request intervention
	else if (ta_action == TAValues::REQUEST) {
        // std::cout << "action : REQUEST" << std::endl;
        state_curr.req_target = target_idx;

		// request to the same target or started to request
		if (state_prev.req_target == target_idx || state_prev.req_time == 0) {
            state_curr.req_time += _delta_t;
		} 
		// change request target 
		else {
			state_curr.req_time = _delta_t;
		}

        obs = _operator_model->execIntervention(state_curr.req_time, state_curr.risk_bin[state_curr.req_target], rand_num, state_curr.risk_type[state_curr.req_target]);
        state_curr.ego_recog[state_curr.req_target] = obs;

	}

	reward = CalcReward(state_prev, state_curr, action);

	if (state_curr.ego_pose >= _planning_horizon)
		return true;
	else
		return false;
}


double TaskAllocation::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {

    int target_idx = _ta_values->getActionTarget(action);
    TAValues::ACT ta_action = _ta_values->getActionAttrib(action);
    const TAState& ras_state = static_cast<const TAState&>(state);

    if (ta_action == TAValues::NO_ACTION) {
        return obs == TAValues::RISK;
    }

    else {
        double acc = _operator_model->intAcc(ras_state.req_time, ras_state.risk_type[ras_state.req_target]);
        return (ras_state.risk_bin[ras_state.req_target] == obs) ? acc : 1.0 - acc;
    }
}


double TaskAllocation::CalcReward(const State& _state_prev, const State& _state_curr, const ACT_TYPE& action) const {
	const TAState& state_prev = static_cast<const TAState&>(_state_prev);
	const TAState& state_curr = static_cast<const TAState&>(_state_curr);
	double reward = 0.0;

    int action_target_idx = _ta_values->getActionTarget(action);
    TAValues::ACT ta_action = _ta_values->getActionAttrib(action);

	for (auto it=state_curr.risk_pose.begin(), end=state_curr.risk_pose.end(); it != end; ++it) {
		if (state_prev.ego_pose < *it && *it <= state_curr.ego_pose) {
            int passed_index = distance(state_curr.risk_pose.begin(), it);


            if (state_curr.risk_bin[passed_index] == TAValues::RISK) {
                reward += (state_curr.ego_speed - _vehicle_model->_yield_speed)/(_vehicle_model->_max_speed - _vehicle_model->_yield_speed) * -100.0;
                // reward += (_max_speed - state_prev.ego_speed)/(_max_speed - _yield_speed) * 100;
            }
            /* driving efficiency */
            else {
                reward += (_vehicle_model->_max_speed - state_prev.ego_speed)/(_vehicle_model->_max_speed - _vehicle_model->_yield_speed) * -10.0;
                // reward += (state_curr.ego_speed - _yield_speed)/(_max_speed - _yield_speed) * 10;
            }

            /* avoid cheating planner by requesting and change recog in last minute */
            // if (state_curr.ego_recog[passed_index] == TAValues::RISK) {
            //     reward += (state_curr.ego_speed - _yield_speed)/(_max_speed - _yield_speed) * -100;
            // }
            
		}
	}

	
    // driving efficiency
    // reward += (state_curr.ego_speed - _yield_speed)/(_max_speed - _yield_speed) * 1;

    // driving comfort
    double deceleration = (state_curr.ego_speed < state_prev.ego_speed) ? (state_curr.ego_speed - state_prev.ego_speed)/_delta_t : 0.0;
    reward += (deceleration < -_vehicle_model->_max_decel) ? -100.0 : 0.0;

    // if (ta_action == TAValues::RECOG && state_prev.risk_bin[action_target_idx] != state_prev.ego_recog[action_target_idx])
    //     reward += -100;

	// int request
	// if (ta_action == TAValues::REQUEST) {

	// if (ta_action == TAValues::REQUEST) {
    //     reward += 1 * -1;
	// }
    
    // penalty for initial intervention request
	if (ta_action == TAValues::REQUEST && (state_curr.req_time == _delta_t || state_prev.req_target != state_curr.req_target)) {
        reward += -0.1;
    }

    /* end of the request */
    if (state_prev.req_time > 0 && (ta_action != TAValues::REQUEST || state_curr.req_target != state_prev.req_target)) {
        /* when operator took mistake <- if no penalty, bet 0.5 of obs = no-risk if it want to keep speed*/
        if (state_curr.risk_bin[state_prev.req_target] != state_curr.ego_recog[state_prev.req_target]) {
            reward += -100.0;
        }
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
    return _start_state;
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
			_ego_recog.emplace_back((ta_start_state->ego_recog[idx] < _risk_thresh) ? false : true);
			if (*col) {
				prob *= _recog_likelihood[idx]; 
				// prob *= 0.5 ; 
				_risk_bin.emplace_back(true);
			}
			else {
				prob *= 1.0 - _recog_likelihood[idx]; 
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
	  	p->risk_type = ta_start_state->risk_type;
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
	  	p->risk_type = ta_start_state->risk_type;
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
    _start_state = static_cast<TAState*>(state);
    
//    // target number up to 3
//    if (_start_state->risk_pose.size() > _max_perception_num) {
//        std::vector<double> pose_list;
//        for (const auto pose : _start_state->risk_pose) {
//            pose_list.emplace_back(pose);
//        }
//        std::sort(pose_list.begin(), pose_list.end());
//        
//        // remove targets farther than the 4th target
//        _planning_horizon = pose_list[_max_perception_num];
//        for (auto i=0; i<_start_state->risk_pose.size();) {
//            if (_start_state->risk_pose[i] >= _planning_horizon) {
//                _start_state->risk_pose.erase(_start_state->risk_pose.begin() + i);
//                _start_state->risk_bin.erase(_start_state->risk_bin.begin() + i);
//                _start_state->ego_recog.erase(_start_state->ego_recog.begin() + i);
//                likelihood_list.erase(likelihood_list.begin() + i);
//            }
//            else {
//                ++i;
//            }
//        }
//    }
//    else {
//        _planning_horizon = 150;
//    }

    _ta_values = new TAValues(_start_state->risk_pose.size());
}

std::vector<double> TaskAllocation::getRiskProb(const Belief* belief) {
	const vector<State*>& particles = static_cast<const ParticleBelief*>(belief)->particles();
	
	// double status = 0;
	vector<double> probs(_start_state->risk_pose.size(), 0.0);
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
 		<< "risk_pose : " << ras_state.risk_pose << "\n"
 		<< "req_target : " << ras_state.req_target << "\n"
 		<< "risk_bin : " << ras_state.risk_bin << "\n"
        << "weight : " << ras_state.weight << "\n"
	 	<< endl;
}

void TaskAllocation::PrintObs(const State& state, OBS_TYPE obs, ostream& out) const {
    switch(obs) {
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
	vector<double> probs(_start_state->risk_pose.size());
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
        const TAState* state = static_cast<const TAState*>(particle);
		for (auto itr=state->risk_bin.begin(), end=state->risk_bin.end(); itr!=end; itr++) {
			probs[distance(state->risk_bin.begin(), itr)] += *itr * particle-> weight;
		}
	}

	for (int i = 0; i < _start_state->risk_pose.size(); i++) {
		out << "risk id : " << i << " prob : " << probs[i] << endl;
	}
}

void TaskAllocation::PrintAction(ACT_TYPE action, ostream& out) const {
    int target_idx = _ta_values->getActionTarget(action);
    TAValues::ACT ta_action = _ta_values->getActionAttrib(action);

	if (ta_action == TAValues::REQUEST)
		out << "request to " << target_idx << endl;
	else
		out << "nothing" << endl;
}
}
