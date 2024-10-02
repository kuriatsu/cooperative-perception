#pragma once

#include "despot/interface/pomdp.h"
#include "despot/core/mdp.h"
#include "cooperative_perception/operator_model.hpp"
#include "cooperative_perception/libgeometry.hpp"
#include "cooperative_perception/vehicle_model.hpp"

namespace despot {

class CPPOMDP: public DSPOMDP {
protected:
	mutable MemoryPool<CPState>      memory_pool;
	// std::vector<CPState*>            states;
	// mutable std::vector<ValuedAction> mdp_policy;
	// OperatorModel                     operator_model;
	
private:
	// reward
	int r_false_positive = -50;
	int r_false_negative = -100;
	int r_eff            = 10;
	int r_comf           = -1;
	int r_request        = -1;

    int max_perception_num_ = 3;

	// state transition parameter
	int    planning_horizon_;
	double yield_speed_;
	double max_speed_;
	double risk_thresh_;
    double delta_t_;


public:
    CPPOMDP ();
    CPPOMDP (const int planning_horizon, const double risk_thresh, const double delta_t, VehicleModel* vehicle_model, OperatorModel* operator_model, State* state); 


    VehicleModel* vehicle_model_;
    OperatorModel* operator_model_;
    // CPState* cp_state_;
    CPValues* cp_values_;
	// recognition likelihood of the ADSbelief(belief);::vector<double> risk_recog;
    // std::vector<double> risk_likelihood_;
	// std::vector<int> risk_positions_;


public:
	// Essential
	int NumActions() const;
	bool Step (State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const;
	double ObsProb (OBS_TYPE obs, const State& state, ACT_TYPE action) const;
	Belief* InitialBelief (const State* start, std::string type = "DEFAULT") const;
	Belief* InitialBelief (const State* start, const std::vector<double>& likelihood, std::string type = "DEFAULT") const;

	double GetMaxReward () const;
	ValuedAction GetBestAction () const;
    std::vector<double> GetPerceptionLikelihood (const Belief* belief);
    ScenarioUpperBound* CreateScenarioUpperBound (std::string name, std::string particle_bound_name) const; 
    ScenarioLowerBound* CreateScenarioLowerBound (std::string name, std::string particle_bound_name) const;

	// Optional
	// ScenarioUpperBound* CreateScenarioUpperBound(std::string name="DEFAULT", std::string particle_bound_name = "DEFAULT") const;
 	// ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT", std::string particle_bound_name = "DEFAULT") const;

	State* Allocate (int state_id, double weight) const;
	State* Copy (const State* particle) const;
	void Free (State* particle) const;
	int NumActiveParticles () const;

    // void syncCurrentState(State* state, std::vector<double>& likelihood_list);
	void PrintState (const State& state, std::ostream& out = std::cout) const;
	void PrintBelief (const Belief& belief, std::ostream& out = std::cout) const;
	void PrintObs (const State& state, OBS_TYPE observation, std::ostream& out = std::cout) const;
	void PrintAction (ACT_TYPE action, std::ostream& out = std::cout) const;

protected:
	void EgoVehicleTransition (int& pose, double& speed, const std::vector<bool>& recog_list, const std::vector<int>& target_poses, const ACT_TYPE& action) const ;
	int CalcReward (const State& state_prev, const State& state_curr, const ACT_TYPE& action) const;
    void GetBinProduct (std::vector<std::vector<bool>>& out_list, std::vector<bool> buf, int row) const ;
};

} // namespace despot


	

        
