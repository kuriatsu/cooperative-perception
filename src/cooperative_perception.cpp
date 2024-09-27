#include "cooperative_perception/cooperative_perception.hpp"

CooperativePerception::CooperativePerception()
{
}

int CooperativePerception::RunPlanning(int argc, char* argv[]) 
{
    // models
    operator_model_ = new OperatorModel();
    vehicle_model_ = new VehicleModel(_delta_t);

    bool search_solver;
    int num_runs = 1;
    string world_type = "simulator";
    string belief_type = "DEFAULT";
    int time_limit = -1;

    option::Option *options = InitializeParamers(argc, argv, solver_type_,
                search_solver, num_runs, world_type, belief_type, time_limit);
    if(options==nullptr)
        return 0;
    clock_t main_clock_start = clock();

    DSPOMDP *model = InitializeModel(options);
    assert(model != nullptr);
    // model_ = new CPPOMDP(planning_horizon_, risk_thresh_, vehicle_model_, operator_model_, delta_t_);

    world_ = InitializeWorld("simulator", model_, options_);
    assert(world != nullptr);

    Belief *belief = nullptr;
    Solver *solver = nullptr;
    Logger *logger = nullptr;

    InitializeLogger(logger, options, model, belief, solver, num_runs,
            main_clock_start, world, world_type, time_limit, policy_type_);

    DisplayParameters(options_, model_);

    logger_->InitRound(world->GetCurrentState());
    round_=0; step_=0;
    PlanningLoop(solver, world, model, logger);
    logger_->EndRound();

    delete world;
    PrintResult(1, logger_, main_clock_start);

    return 0;
}



void CooperativePerception::PlanningLoop(Solver*& solver, World* world, DSPOMDP* model, Logger* logger) 
{
    bool terminal = false;
    while (!terminal) {
        terminal = RunStep(solver, world, model, logger);
    }
}

bool CooperativePerception::RunStep(State* solver, World* world, DSPOMDP* model, Logger* logger)
{

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

    cp_world->UpdatePerception(cp_model->GetPerceptionLikelihood(belief));

    return logger->SummarizeStep(step_++, round_, terminal, action, obs, step_start_t);
}


void CooperativePerception::InitializeWorld(int argc, char* argv[])
{
    std::shared_ptr<CPWorld> world = std::make_shared<CPWorld>(argc, argv);
    world->Connect();
    world->Initialize();
    return world;
}

std::string CooperativePerception::ChooseSolver() {
    return "DESPOT";
}

void CooperativePerception::InitializeDefaultParameters() 
{
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
    return CooperativePerception().RunPlanning(argc, argv);
}
