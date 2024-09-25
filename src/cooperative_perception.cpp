#include "cooperative_perception_planner/cooperative_perception.hpp"


CooperativePerceptionPlanner::~CooperativePerceptionPlanner() {
}

int CooperativePerceptionPlanner::RunPlanning(int argc, char* argv[]) 
{
    // models
    _operator_model = new OperatorModel();
    _vehicle_model = new VehicleModel(_delta_t);

    bool search_solver;
    int num_runs = 1;
    string world_type = "simulator";
    string belief_type = "DEFAULT";
    int time_limit = -1;

    option::Option *options = InitializeParamers(argc, argv, _solver_type,
                search_solver, num_runs, world_type, belief_type, time_limit);
    if(options==nullptr)
        return 0;
    clock_t main_clock_start = clock();

    DSPOMDP *model = InitializeModel(options);
    assert(model != nullptr);
    // _model = new CPPOMDP(_planning_horizon, _risk_thresh, _vehicle_model, _operator_model, _delta_t);

    _world = InitializeWorld("simulator", _model, _options);
    assert(world != nullptr);

    Belief *belief = nullptr;
    Solver *solver = nullptr;
    Logger *logger = nullptr;

    InitializeLogger(logger, options, model, belief, solver, num_runs,
            main_clock_start, world, world_type, time_limit, _policy_type);

    DisplayParameters(_options, _model);

    _logger->InitRound(world->GetCurrentState());
    round_=0; step_=0;
    PlanningLoop(solver, world, model, logger);
    _logger->EndRound();

    delete world;
    PrintResult(1, _logger, main_clock_start);

    return 0;
}


void CooperativePerceptionPlanner::PlanningLoop(Solver*& solver, World* world, DSPOMDP* model, Logger* logger) {
    bool terminal = false;
    while (!terminal) {
        terminal = RunStep(solver, world, model, logger);
    }
}

bool CooperativePerceptionPlanner::RunStep(State* solver, World* world, DSPOMDP* model, Logger* logger) {

    logger->CheckTargetTime();
    double step_start_t = get_time_second();

    CPWorld* cp_world = static_cast<CPWorld*>(world);
    CPPOMDP* cp_model = static_cast<CPPOMDP*>(model);

    if (cp_world->IsTerminate())
    {
        return true;
    }

    std::vector<double> likelihood_list;
    std::shared_ptr<CPState> *state = std::make_shared<CPState>();
    cp_world->GetCurrentState(state, likelihood_list)

    Belief* belief = cp_model->InitialBelief(state, likelihood_list, belief_type);
    assert(belief != NULL);
    // solver->belief(belief);

    solver = InitializeSolver(model, belief, "DESPOT", options);

    double start_t = get_time_second();

    ACT_TYPE action;
    if (policy_type == "MYOPIC")
        action = MyopicAction();
    else if (policy_type == "EGOISTIC")
        action = EgoisticAction();
    else
        action = solver->Search().action;

    double end_t = get_time_second();
    double search_time = end_t - start_t;

    start_t = get_time_second();
    OBS_TYPE obs;
    bool terminal = cp_world->ExecuteAction(action, obs);
    end_t = get_time_second();
    double execute_time = end_t - start_t;

    
    start_t = get_time_second();
    solver->BeliefUpdate(action, obs);
    end_t = get_time_second();
    double update_time = end_t - start_t;

    cp_world->SyncBelief(cp_world->GetRiskProb(belief));

    return logger->SummarizeStep(step_++, round_, terminal, action, obs, step_start_t);
}


void CooperativePerceptionPlanner::InitializeWorld(int argc, char* argv[])
{
    std::shared_ptr<CPWorld> world = std::make_shared<CPWorld>();
    world->Connect(argc, argv);
    world->Initialize();
    return world;
}


void CooperativePerceptionPlanner::InitializeDefaultParameters() {
    Globals::config.num_scenarios = 100;
    Globals::config.sim_len = 90;
    Globals::config.time_per_move = 2.0; 
}


DSPOMDP* InitializeModel(option::Option* options)
{
    std::shared_ptr<DSPOMDP> model = std::make_shared<CPPOMDP>();
    return model;
}


int main(int argc, char ** argv)
{
    return CooperativePerceptionPlanner::RunPlanning(argc, argv);
}
