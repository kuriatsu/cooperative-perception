
DSPOMDP* CooperativePerceptionPlanner::InitializeModel(option::Option* options) {
    DSPOMDP* model = new TaskAllocation(planning_horizon, risk_thresh, vehicle_model, operator_model, delta_t);
    return model;
}

World* CooperativePerceptionPlanner::InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options) {
    RasWorld* ras_world = new RasWorld(vehicle_model, operator_model, delta_t, obstacle_density, perception_range, policy_type+std::to_string(obstacle_density)+"_");
    ras_world->Connect();
    ras_world->Initialize();
    return ras_world;
}

void CooperativePerceptionPlanner::InitializeDefaultParameters() {
    Globals::config.num_scenarios = 100;
    Globals::config.sim_len = 90;
    Globals::config.time_per_move = 2.0; 
}


void CooperativePerceptionPlanner::PredictedObjectsCb(autoware_perception_msgs::msg::PredictedObjectsUE &msg) {
    RasWorld* ras_world = static_cast<RasWorld*>(world);
    TaskAllocation* ta_model = static_cast<TaskAllocation*>(model);
    logger->CheckTargetTime();
    
    for (int i=0; i<delta_t; i++) {
        ras_world->Step(0);
    }

    if (ras_world->isTerminate()) {
        return true;
    }

    TAState* start_state = static_cast<TAState*>(_current_state);
    std::vector<double> likelihood_list = ras_world->GetPerceptionLikelihood(_current_state);
    ta_model->syncCurrentState(start_state, likelihood_list);
    RunStep(solver, world, model, logger);
}


bool CooperativePerceptionPlanner::RunStep(State* solver, World* world, DSPOMDP* model, Logger* logger) {

    Belief* belief = ta_model->InitialBelief(start_state, likelihood_list, belief_type);
    assert(belief != NULL);
    // solver->belief(belief);

    solver = InitializeSolver(model, belief, "DESPOT", options);

    double step_start_t = get_time_second();
    double start_t = get_time_second();

    ACT_TYPE action;
    if (policy_type == "MYOPIC")
        action = ras_world->MyopicAction();
    else if (policy_type == "EGOISTIC")
        action = ras_world->EgoisticAction();
    else
        action = solver->Search().action;

    double end_t = get_time_second();
    double search_time = end_t - start_t;

    OBS_TYPE obs;
    start_t = get_time_second();
    bool terminal = ras_world->ExecuteAction(action, obs);
    end_t = get_time_second();
    double execute_time = end_t - start_t;

    start_t = get_time_second();
    solver->BeliefUpdate(action, obs);
    end_t = get_time_second();
    double update_time = end_t - start_t;

    ras_world->UpdateState(action, obs, ta_model->getRiskProb(belief));
    ras_world->Log(action, obs);

    return false;
}


