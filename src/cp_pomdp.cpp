#include "cooperative_perception/cp_pomdp.hpp"

#include "despot/core/builtin_lower_bounds.h"
#include "despot/core/builtin_policy.h"
#include "despot/core/builtin_upper_bounds.h"
#include "despot/core/particle_belief.h"

using namespace std;

namespace despot {

class CPDefaultPolicy: public DefaultPolicy {
protected:
    const CPPOMDP* cp_model;
    const VehicleModel* vehicle_model;
    const OperatorModel* operator_model;
    const CPValues* cp_values;
    
public:
    CPDefaultPolicy(const CPPOMDP* model, ParticleLowerBound* bound) : 
        DefaultPolicy(model, bound),
        cp_model(model), 
        vehicle_model(model->vehicle_model_),
        operator_model(model->operator_model_),
        cp_values(model->cp_values_) {
        }

    ACT_TYPE Action(const vector<State*>& particles, RandomStreams& streams, History& history) const {
        const CPState& cp_state = static_cast<const CPState&>(*particles[0]);

        if (history.Size()) {
            ACT_TYPE action = history.LastAction();
            OBS_TYPE obs = history.LastObservation();

            if (cp_state.req_time > 0) { 

                if (cp_model->operator_model_->InterventionAccuracy(cp_state.req_time) <= 0.5) {
                    return cp_values->getAction(CPValues::REQUEST, cp_state.req_target);
                }
                // else if (cp_model->operator_model_->InterventionAccuracy(cp_state.req_time) == 1.0) {
                //    if (obs != cp_state.ego_recog[cp_state.req_target]) {
                //        return cp_values->getAction(CPValues::RECOG, cp_state.req_target);
                //    }
                //}
            }
            
//            else {
//                double comf_stop_dist = vehicle_model->getDecelDistance(cp_state.ego_speed, vehicle_model->min_decel_, vehicle_model->safety_margin_);
//                double harsh_stop_dist = vehicle_model->getDecelDistance(cp_state.ego_speed, vehicle_model->max_decel_, 0.0);
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
////                            if (history.Action(j) == cp_values->getAction(CPValues::RECOG, i)) {
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
//                            if (history.Action(j) == cp_values->getAction(CPValues::REQUEST, i)) {
//                                // std::cout << "already in hist : " << i << std::endl;
//                                in_history = true;
//                                break;
//                            }
//                        }
//                        if (!in_history)
//                            request_target_list.emplace_back(i); 
//                    }
//                    if (cp_state.ego_recog[i] == CPValues::NO_RISK) {
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
//                    return cp_model->cp_values_->getAction(CPValues::NO_ACTION, 0);
//                }
//
//                else if (recog_target_list.empty()) {
//                    // std::cout <<  "request selected : " << request_target << std::endl;
//                    return cp_values->getAction(CPValues::REQUEST, request_target);
//                }
//
//                else if (request_target_list.empty()) {
//                    // std::cout <<  "recog selected : " << recog_target << std::endl;
//                    return cp_values->getAction(CPValues::RECOG, recog_target);
//                }
//                else if (cp_state.risk_pose[request_target] < cp_state.risk_pose[recog_target]) {
//                    // std::cout <<  "request selected : " << request_target << std::endl;
//                    return cp_values->getAction(CPValues::REQUEST, request_target);
//                }
//                else {
//                    // std::cout <<  "recog selected : " << recog_target << std::endl;
//                    return cp_values->getAction(CPValues::RECOG, recog_target);
//                }
//            }

        }
        return cp_model->cp_values_->getAction(CPValues::NO_ACTION, 0);
    }
};


// class CPParticleUpperBound: public ParticleUpperBound {
// protected:
//     const CPPOMDP* cp_model;
// public:
//     CPUpperBound(const CPPOMDP* model) : 
//         cp_model(model) {
//     }
// 
//     double Value(const State& state) const {
//         const CPState& cp_state = static_cast<const CPState&>(state);
        
CPPOMDP::CPPOMDP(int planning_horizon, double risk_thresh, VehicleModel* vehicle_model, OperatorModel* operator_model, double delta_t){ 
    planning_horizon_ = planning_horizon;
    risk_thresh_ = risk_thresh; 
    vehicle_model_ = vehicle_model;
    operator_model_ = operator_model;
    max_speed_ = vehicle_model_->max_speed_;
    yield_speed_ = vehicle_model_->yield_speed_;
    delta_t_ = delta_t;
}

CPPOMDP::CPPOMDP() {
    planning_horizon_ = 150;
    risk_thresh_ = 0.5; 
    // delta_t_ = Globals::config.time_per_move;
    delta_t_ = 2.0;

    vehicle_model_ = new VehicleModel();
    operator_model_ = new OperatorModel();
    cp_state_ = new CPState();
    cp_values_ = new CPValues(cp_state_->risk_pose.size());

    vehicle_model_->delta_t_ = delta_t_;
    max_speed_ = vehicle_model_->max_speed_;
    yield_speed_ = vehicle_model_->yield_speed_;

    risk_likelihood_ = {0.4, 0.4, 0.6};
}

CPPOMDP::CPPOMDP(const double delta_t, const std::vector<int> risk_pose, const std::vector<double> risk_likelihood)
    : delta_t_(delta_t),
      risk_likelihood_(risk_likelihood)
    {
    planning_horizon_ = 150;
    risk_thresh_ = 0.5; 

    vehicle_model_ = new VehicleModel();
    operator_model_ = new OperatorModel();

    std::vector<bool> risk_bin;
    for (const auto likelihood : risk_likelihood_) {
        risk_bin.emplace_back((likelihood > risk_thresh_) ? true : false);
    }

    cp_state_ = new CPState(0.0, max_speed_, risk_bin, 0, 0, risk_bin, risk_pose);
    cp_values_ = new CPValues(cp_state_->risk_pose.size());

    vehicle_model_->delta_t_ = delta_t_;
    max_speed_ = vehicle_model_->max_speed_;
    yield_speed_ = vehicle_model_->yield_speed_;

}

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
        return new CPDefaultPolicy(this, CreateParticleLowerBound(particle_bound_name));
    }
    else {
        std::cerr << "Unsupported lower bound: " << name << std::endl;
        exit(0);
        return NULL;
    }
}



int CPPOMDP::NumActions() const {
	return 1 + cp_state_->risk_pose.size() * 2;
}

bool CPPOMDP::Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs)  const {
	CPState& state_curr = static_cast<CPState&>(state);
	CPState state_prev = state_curr;
	reward = 0.0;

	// ego state trantion
	// EgoVehicleTransition(state_curr.ego_pose, state_curr.ego_speed, state_prev.ego_recog, risk_pose, action);
    vehicle_model_->GetTransition(state_curr.ego_speed, state_curr.ego_pose, state_prev.ego_recog, state_prev.risk_pose);
    
    int target_idx = cp_values_->getActionTarget(action);
    CPValues::ACT cp_action = cp_values_->getActionAttrib(action);
    
    // when action == no_action
    if (cp_action == CPValues::NO_ACTION) {
        // std::cout << "action : NO_ACTION" << std::endl;
        state_curr.req_time = 0;
        state_curr.req_target = 0;
        obs = operator_model_->ExecIntervention(state_curr.req_time, cp_action, "", CPValues::NO_RISK);
    }
    
	// when action = change recog state
    // else if (cp_action == CPValues::RECOG) {
        // std::cout << "action : RECOG" << std::endl;
	// 	state_curr.ego_recog[target_idx] = (state_prev.ego_recog[target_idx] == CPValues::RISK) ? CPValues::NO_RISK : CPValues::RISK;
    //     state_curr.req_time = 0;
    //     state_curr.req_target = 0;
    //     obs = operator_model_->ExecIntervention(state_curr.req_time, cp_action, "", CPValues::NO_RISK);
	// }
    
	// when action = request intervention
	else if (cp_action == CPValues::REQUEST) {
        // std::cout << "action : REQUEST" << std::endl;
        state_curr.ego_recog[target_idx] = CPValues::RISK;

		// request to the same target
		if (state_prev.req_target == target_idx) {
            state_curr.req_time += delta_t_;
			state_curr.req_target = target_idx;
		} 
		// request to new target
		else {
			state_curr.req_time = delta_t_;
			state_curr.req_target = target_idx;
		}

        obs = operator_model_->ExecIntervention(state_curr.req_time, cp_action, std::to_string(target_idx), state_curr.risk_bin[target_idx]);

	}
	reward = CalcReward(state_prev, state_curr, action);

	if (state_curr.ego_pose >= planning_horizon_)
		return true;
	else
		return false;
}


double CPPOMDP::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {

    int target_idx = cp_values_->getActionTarget(action);
    CPValues::ACT cp_action = cp_values_->getActionAttrib(action);

    if (cp_action == CPValues::REQUEST) {
        const CPState& ras_state = static_cast<const CPState&>(state);
        double acc = operator_model_->InterventionAccuracy(ras_state.req_time);

        if (obs == CPValues::NONE) return 0.0;
        return (ras_state.risk_bin[target_idx] == obs) ? acc : 1.0 - acc;
    }
    else {
        return obs == CPValues::NONE;
        // return 1.0;
    }
}


int CPPOMDP::CalcReward(const State& _state_prev, const State& _state_curr, const ACT_TYPE& action) const {
	const CPState& state_prev = static_cast<const CPState&>(_state_prev);
	const CPState& state_curr = static_cast<const CPState&>(_state_curr);
	int reward = 0;

    int action_target_idx = cp_values_->getActionTarget(action);
    CPValues::ACT cp_action = cp_values_->getActionAttrib(action);

	for (auto it=state_curr.risk_pose.begin(), end=state_curr.risk_pose.end(); it != end; ++it) {
		if (state_prev.ego_pose <= *it && *it < state_curr.ego_pose) {
            int passed_index = distance(state_curr.risk_pose.begin(), it);

			// driving safety
            reward += (state_prev.risk_bin[passed_index] == state_prev.ego_recog[passed_index]) ? 10 : -10;
            if (state_prev.risk_bin[passed_index] == CPValues::RISK) {
                reward += (state_prev.ego_speed - yield_speed_)/(max_speed_ - yield_speed_) * -100;
            }
            else {
                reward += (state_prev.ego_speed - yield_speed_)/(max_speed_ - yield_speed_) * 10;
            }
		}
	}

	if (cp_action == CPValues::REQUEST && (state_curr.req_time == delta_t_ || state_prev.req_target != state_curr.req_target)) {
        reward += 1 * -1 ;
	}

	return reward;
}

double CPPOMDP::GetMaxReward() const {
	return 1000;
}

ValuedAction CPPOMDP::GetBestAction() const {
	return ValuedAction(CPValues::NO_ACTION, 0);
}


State* CPPOMDP::CreateStartState(string type) const {
    return cp_state_;
}

Belief* CPPOMDP::InitialBelief(const State* start, std::string type) const {
   
    const CPState *cp_start_state = static_cast<const CPState*>(start);

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
			_ego_recog.emplace_back((cp_start_state->ego_recog[idx] < risk_thresh_) ? false : true);
			if (*col) {
				prob *= risk_likelihood_[idx]; 
				// prob *= 0.5 ; 
				_risk_bin.emplace_back(true);
			}
			else {
				prob *= 1.0 - risk_likelihood_[idx]; 
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

std::vector<double> CPPOMDP::GetPerceptionLikelihood(const Belief* belief) {
	const vector<State*>& particles = static_cast<const ParticleBelief*>(belief)->particles();
	
	// double status = 0;
	vector<double> probs(cp_state_->risk_pose.size(), 0.0);
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		CPState* state = static_cast<CPState*>(particle);
		for (auto itr=state->risk_bin.begin(), end=state->risk_bin.end(); itr!=end; itr++) {
			probs[distance(state->risk_bin.begin(), itr)] += *itr * particle->weight;
		}
	}
    return probs;
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
//     cp_state_ = static_cast<CPState*>(state);
    
//    // target number up to 3
//    if (cp_state_->risk_pose.size() > max_perception_num_) {
//        std::vector<double> pose_list;
//        for (const auto pose : cp_state_->risk_pose) {
//            pose_list.emplace_back(pose);
//        }
//        std::sort(pose_list.begin(), pose_list.end());
//        
//        // remove targets farther than the 4th target
//        planning_horizon_ = pose_list[max_perception_num_];
//        for (auto i=0; i<cp_state_->risk_pose.size();) {
//            if (cp_state_->risk_pose[i] >= planning_horizon_) {
//                cp_state_->risk_pose.erase(cp_state_->risk_pose.begin() + i);
//                cp_state_->risk_bin.erase(cp_state_->risk_bin.begin() + i);
//                cp_state_->ego_recog.erase(cp_state_->ego_recog.begin() + i);
//                likelihood_list.erase(likelihood_list.begin() + i);
//            }
//            else {
//                ++i;
//            }
//        }
//    }
//    else {
//        planning_horizon_ = 150;
//    }

//     cp_values_ = new CPValues(cp_state_->risk_pose.size());
// }


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
        case CPValues::NONE:
            out << "NONE" << endl;
            break;
        case CPValues::NO_RISK:
            out << "NO_RISK" << endl;
            break;
        case CPValues::RISK:
            out << "RISK" << endl;
            break;
    }
}

void CPPOMDP::PrintBelief(const Belief& belief, ostream& out) const {
	const vector<State*>& particles = static_cast<const ParticleBelief&>(belief).particles();
	
	// double status = 0;
	vector<double> probs(cp_state_->risk_pose.size());
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
        const CPState* state = static_cast<const CPState*>(particle);
		for (auto itr=state->risk_bin.begin(), end=state->risk_bin.end(); itr!=end; itr++) {
			probs[distance(state->risk_bin.begin(), itr)] += *itr * particle-> weight;
		}
	}

	for (int i = 0; i < cp_state_->risk_pose.size(); i++) {
		out << "risk id : " << i << " prob : " << probs[i] << endl;
	}
}

void CPPOMDP::PrintAction(ACT_TYPE action, ostream& out) const {
    int target_idx = cp_values_->getActionTarget(action);
    CPValues::ACT cp_action = cp_values_->getActionAttrib(action);

	if (cp_action == CPValues::REQUEST)
		out << "request to " << target_idx << endl;
	else
		out << "nothing" << endl;
}
}
