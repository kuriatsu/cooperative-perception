#pragma once

#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include "operator_model.h"

namespace despot {

class RasState : public State {
public:
	int ego_pose;
    double ego_speed;
	std::vector<bool> ego_recog;
	int req_time;
	int req_target;

	// hidden state
	std::vector<bool> risk_bin;

    RasState();
    RasState(int _ego_pose, float _ego_speed, std::vector<bool> _ego_recog, int _req_time, int _req_target, std::vector<bool> _risk_bin) :
		ego_pose(_ego_pose),
		ego_speed(_ego_speed),
		ego_recog(_ego_recog),
		req_time(_req_time),
		req_target(_req_target),
		risk_bin(_risk_bin)	{
        }
	~RasState();
	
	std::string text() const;
};

class Ras: public DSPOMDP {
protected:
	mutable MemoryPool<RasState> memory_pool;
	std::vector<RasState*> states;
	mutable std::vector<ValuedAction> mdp_policy;
	OperatorModel operator_model;
	
private:
	// reward
	int r_false_positive = -500;
	int r_false_negative = -1000;
	int r_eff = -1000;
	int r_comf = -1;
	int r_request = -1;

public:
	// state transition parameter
	int planning_horizon;
	double yield_speed;
	double ideal_speed;
	double ordinary_G;
	int safety_margin;
	double risk_thresh;

	// recognition likelihood of the ADSbelief(belief);::vector<double> risk_recog;
	std::vector<Risk> risks;
	
	
	enum { NO_ACTION = 2*2, REQUEST = 0, RECOG = 2 }; // action TODO define based on the given situation
	// enum { NO_ACTION = risk_pose.size()*2, REQUEST = 0, RECOG = risk_pose.size() }; // action
	enum { NO_TARGET = 2 }; // req_target
	// enum { NO_TARGET = risk_pose.size()} // req_target TODO define based on the given situation
	enum { NO_RISK = 0, RISK = 1, NONE = 2}; // risk_state, ego_recognition

public:
	Ras(int planning_horizon, double ideal_speed, double yield_speed, double ordinary_G, double safety_margin, double risk_thresh);
    Ras::Ras() :
        planning_horizon(150),
        ideal_speed(11.2),
        yield_speed(2.8),
        ordinary_G(0.2),
        safety_margin(5),
        risk_thresh(0.5){ 
        }

	// Essential
	int NumActions() const;
	bool Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const;
	double ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const;
	State* CreateStartState(std::string type="DEFAULT") const;
	Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

	double GetMaxReward() const;
	ValuedAction GetBestAction() const;

	// Optional
	// ScenarioUpperBound* CreateScenarioUpperBound(std::string name="DEFAULT", std::string particle_bound_name = "DEFAULT") const;
	// ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT", std::string particle_bound_name = "DEFAULT") const;

	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

	void PrintState(const State& state, std::ostream& out = std::cout) const;
	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	void PrintObs(const State& state, OBS_TYPE observation, std::ostream& out = std::cout) const;
	void PrintAction(ACT_TYPE action, std::ostream& out = std::cout) const;

protected:
	void EgoVehicleTransition(int& pose, double& speed, const std::vector<bool>& recog_list, const std::vector<int>& target_poses, const ACT_TYPE& action) const ;
	int CalcReward(const State& state_prev, const State& state_curr, const std::vector<int>& risk_poses, const ACT_TYPE& action) const;
    void GetBinProduct(std::vector<std::vector<bool>>& out_list, std::vector<bool> buf, int row) const ;
};

} // namespace despot


	

        
