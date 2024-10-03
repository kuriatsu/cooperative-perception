#include "cooperative_perception/cooperative_perception.hpp"

CooperativePerception::CooperativePerception()
{
}

int CooperativePerception::RunPlanning(int argc, char* argv[]) 
{
    // models
    operator_model_ = new OperatorModel();
    vehicle_model_ = new VehicleModel(delta_t_);

    bool search_solver;
    int num_runs = 1;
    string world_type = "simulator";
    int time_limit = -1;

    options_ = InitializeParamers(argc, argv, policy_type_,
                search_solver, num_runs, world_type, belief_type_, time_limit);
    if(options_==nullptr)
        return 0;
    clock_t main_clock_start = clock();

    DSPOMDP *model = new CPPOMDP();
    // DSPOMDP *model = InitializeModel(options_);
    assert(model != nullptr);
    // model_ = new CPPOMDP(planning_horizon_, risk_thresh_, vehicle_model_, operator_model_, delta_t_);

    World* world = InitializeWorld(argc, argv, world_type, model, options_);
    assert(world != nullptr);

    Belief *belief = nullptr;
    Solver *solver = nullptr;
    Logger *logger = nullptr;

    InitializeLogger(logger, options_, model, belief, solver, num_runs,
            main_clock_start, world, world_type, time_limit, policy_type_);

    DisplayParameters(options_, model);

    logger->InitRound(world->GetCurrentState());
    round_=0; step_=0;
    PlanningLoop(solver, world, model, logger);
    logger->EndRound();

    delete world;
    PrintResult(1, logger, main_clock_start);

    return 0;
}

Solver* CooperativePerception::CPInitializeSolver(DSPOMDP *model, Belief *belief, World *world)
{
    if (policy_type_ == "DESPOT") {
        Solver *solver = InitializeSolver(model, belief, policy_type_, options_);
        std::cout << "[cooperative_perception::CPInitializeSolver] initialize solver" << std::endl;
        return solver;

    } else if (policy_type_ == "NOREQUEST") {
        NoRequestModel *solver = new NoRequestModel(model, belief, world);
        solver->belief(belief);
        return solver;

    } else if (policy_type_ == "MYOPIC") {
        MyopicModel *solver = new MyopicModel(model,
                                 belief,
                                 vehicle_model_, 
                                 operator_model_,
                                 world);

        solver->belief(belief);
        return solver;
    }

}


void CooperativePerception::PlanningLoop(Solver*& solver, World* world, DSPOMDP* model, Logger* logger) 
{
    bool terminal = false;
    while (!terminal) {
        terminal = RunStep(solver, world, model, logger);
    }
}

bool CooperativePerception::RunStep(Solver* solver, World* world, DSPOMDP* model, Logger* logger)
{

    logger->CheckTargetTime();
    double step_start_t = get_time_second();

    CPWorld* cp_world = static_cast<CPWorld*>(world);


    std::vector<double> likelihood_list;
    State *state = cp_world->GetCurrentState(likelihood_list, risk_thresh_);
    
    std::cout << static_cast<CPState*>(state)->risk_pose.size() << std::endl;

    std::cout << "[cooperative_perception::RunStep] curent_state: \n" << state->text() << std::endl;

    CPPOMDP* cp_model = InitializeModel(state);

    Belief* belief = cp_model->InitialBelief(state, likelihood_list, belief_type_);
    assert(belief != NULL);
    cp_model->PrintBelief(*belief);
    // solver->belief(belief);

    solver = CPInitializeSolver(cp_model, belief, cp_world);
    std::cout << "[cooperative_perception::RunStep] initialized solver" << std::endl;

    double start_t = get_time_second();
    ACT_TYPE action = solver->Search().action;
    double end_t = get_time_second();
    double search_time = end_t - start_t;
    std::cout << "[cooperative_perception::RunStep] search completed" << std::endl;

    start_t = get_time_second();
    OBS_TYPE obs;
    bool terminal = cp_world->CPExecuteAction(action, obs);
    end_t = get_time_second();
    double execute_time = end_t - start_t;

    
    start_t = get_time_second();
    solver->BeliefUpdate(action, obs);
    end_t = get_time_second();
    double update_time = end_t - start_t;

    cp_world->UpdatePerception(action, obs, cp_model->GetPerceptionLikelihood(belief));
    cp_world->Step();

    return logger->SummarizeStep(step_++, round_, terminal, action, obs, step_start_t);
}


World* CooperativePerception::InitializeWorld(int argc, char* argv[], std::string& world_type, DSPOMDP* model, option::Option* options)
{
    std::cout << "[cooperative_perception::InitializeWorld] initialize world" << std::endl;
    CPWorld* world = new CPWorld();
    world->Connect(argc, argv);
    world->Initialize();
    world->Step();
    return world;
}

    World* CooperativePerception::InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options)
{
    return nullptr;
}

std::string CooperativePerception::ChooseSolver() {
    return policy_type_;
}

void CooperativePerception::InitializeDefaultParameters() 
{
    Globals::config.num_scenarios = 100;
    Globals::config.sim_len = 90;
    Globals::config.time_per_move = 1.0; 
}


DSPOMDP* CooperativePerception::InitializeModel(option::Option* options)
{
    return new CPPOMDP();
}

CPPOMDP* CooperativePerception::InitializeModel (State* state)
{
    CPPOMDP* model = new CPPOMDP(planning_horizon_, risk_thresh_, delta_t_, vehicle_model_, operator_model_, state);
    return model;
}

int main(int argc, char ** argv)
{
    return CooperativePerception().RunPlanning(argc, argv);
}
