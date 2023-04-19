#pragma once

#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>

namespace despot {

class RasState : public State {
public:
	int ego_pose;
    float ego_speed;
	int req_time;
	int req_target;
	int ego_recog;
	std::vector<bool> target_risk;
	std::vector<bool> target_pose;

    RasState():
    RasState(int _ego_pose, float _ego_speed, int _req_time, int _req_target, int _ego_recog, std::vector<int> _target_risk, std::vector<int> _target_pose) :
		ego_pose(_ego_pose),
		ego_speed(_ego_speed),
		ego_recog(_ego_recog),
		req_time(_req_time),
		req_target(_req_target),
		target_risk(_target_risk),
		target_pose(_target_pose) {
		}
	~RasState();
	
	std::string text() const;
};

class Ras: public DSPOMDP {
protected:
	mutable MemoryPool<RasState> memory_pool;
	std::vector<RasState*> states;
	mutable std::vector<ValuedAction> mdp_policy;

public:
	int target_num;
	enum { NO_ACTION = target_num*2, REQUEST = 0, RECOG = target_num }; // action
	enum { NO_INT = 0, INT = 1}; // observation
	enum { NO_TARGET = target_num }; // req_target
	enum { NO_RISK = 0, RISK = 1 }; // risk_state, ego_recognition
	
public:
	RasState();

	int NumActions() const;

	bool Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const;

	double ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const;
	// State* CreateStartState(std::string type="DEFAULT") const;
	Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

	double GetMaxReward() const;
	ScenarioUpperBound* CreateScenarioUpperBound(std::string name="DEFAULT", std::string particle_bound_name = "DEFAULT") const;
	ValuedAction GetBestAction() const;
	ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT", std::string particle_bound_name = "DEFAULT") const;

	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

	void PrintState(const State& state, std::ostream& out = std::cout) const;
	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	void PrintObs(const State& state, OBS_TYPE observation, std::ostream& out = std::cout) const;
	void PrintAction(ACT_TYPE action, std::ostream& out = std::cout) const;
};

} // namespace despot


	

        
