#pragma once
#include "ras.h"
#include "despot/interface/world.h"
#include "sumo_interface.h"

using namespace despot;

class RasWorld: public World {
public:
    SumoInterface sim;

public:
    bool Connect();
    State* Initialize();
    State* GetCurrentState();
    bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);
    ~World();
} 

