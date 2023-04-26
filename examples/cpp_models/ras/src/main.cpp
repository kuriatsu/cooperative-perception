#include <despot/planner.h>
#include "ras.h"

using namespace despot;

class MyPlanner: public Planner {
public:
	MyPlanner() {
	}

	DSPOMDP* InitializeModel(option::Option* options) {
		DSPOMDP* model = new Ras();
		return model;
	}

	World* InitialzeWorld(std::string& world_type, DSPOMDP* model, option::Option* options) {
		return InitializePOMDPWorld(world_type, model, options);
	}

	void InitialzeDefaultParameters() {
	}

	std::string ChooseSolver() {
		return "DESPOT";
	}
};

int main(int argc, char** argv) {
	return MyPlanner().RunEvaluation(argc, argv);
}
