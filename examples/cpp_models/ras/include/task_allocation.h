#pragma once

#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include "operator_model.h"
#include "libgeometry.h"
#include "vehicle_model.h"

namespace despot {

class TAState : public State {
public:
	int ego_pose;
    double ego_speed;
	std::vector<bool> ego_recog;
	int req_time;
	int req_target;
    std::vector<int> risk_pose;

	// hidden state
	std::vector<bool> risk_bin;
    
    TAState();
    TAState(int _ego_pose, float _ego_speed, std::vector<bool> _ego_recog, int _req_time, int _req_target, std::vector<bool> _risk_bin, std::vector<int> _risk_pose); 
	~TAState();
	
	std::string text() const;
};

class TaskAllocation: public DSPOMDP {
protected:
	mutable MemoryPool<TAState>      memory_pool;
	std::vector<TAState*>            states;
	mutable std::vector<ValuedAction> mdp_policy;
	OperatorModel                     operator_model;
	
private:
	// reward
	int r_false_positive = -50;
	int r_false_negative = -100;
	int r_eff            = 10;
	int r_comf           = -1;
	int r_request        = -1;

    int m_max_perception_num = 3;

public:
    TaskAllocation(int planning_horizon, double risk_thresh, VehicleModel* vehicle_model, OperatorModel* operator_model, double delta_t); 
    TaskAllocation(const double delta_t_, const std::vector<int> risk_pose_, const std::vector<double> risk_likelihood_);
    TaskAllocation();

	// state transition parameter
	int    m_planning_horizon;
	double m_yield_speed;
	double m_max_speed;
	double m_risk_thresh;
    double m_delta_t;

	// recognition likelihood of the ADSbelief(belief);::vector<double> risk_recog;
    std::vector<double> m_recog_likelihood;
	// std::vector<int> m_risk_positions;

    VehicleModel* m_vehicle_model;
    OperatorModel* m_operator_model;
    TAState* m_start_state;
    TAValues* m_ta_values;

public:
	// Essential
	int NumActions() const;
	bool Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const;
	double ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const;
	State* CreateStartState(std::string type="DEFAULT") const;
	Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;
	Belief* InitialBelief(const State* start, const std::vector<double>& likelihood, std::string type = "DEFAULT") const;

	double GetMaxReward() const;
	ValuedAction GetBestAction() const;
    ScenarioUpperBound* CreateScenarioUpperBound(std::string name, std::string particle_bound_name) const; 
    ScenarioLowerBound* CreateScenarioLowerBound(std::string name, std::string particle_bound_name) const;

	// Optional
	// ScenarioUpperBound* CreateScenarioUpperBound(std::string name="DEFAULT", std::string particle_bound_name = "DEFAULT") const;
 	// ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT", std::string particle_bound_name = "DEFAULT") const;

	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

    void syncCurrentState(State* state, std::vector<double>& likelihood_list);
    std::vector<double> getRiskProb(const Belief* belief); 
	void PrintState(const State& state, std::ostream& out = std::cout) const;
	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	void PrintObs(const State& state, OBS_TYPE observation, std::ostream& out = std::cout) const;
	void PrintAction(ACT_TYPE action, std::ostream& out = std::cout) const;

protected:
	void EgoVehicleTransition(int& pose, double& speed, const std::vector<bool>& recog_list, const std::vector<int>& target_poses, const ACT_TYPE& action) const ;
	int CalcReward(const State& state_prev, const State& state_curr, const ACT_TYPE& action) const;
    void GetBinProduct(std::vector<std::vector<bool>>& out_list, std::vector<bool> buf, int row) const ;
};

} // namespace despot


	

        
