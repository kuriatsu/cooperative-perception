#include "ras_world.h"

using namespace despot;

~World() {
    Simulation::close();
}

bool RasWorld::Connect(){
    Simulation::start({"sumo-gui", "-c", "straight.sumocfg"});
    // Simulation::start({"sumo-gui", "-r", "./straight.net.xml"});
    sim = SumoInterface();
    sim.spawnPedestrians();
    sim.spawnEgoVehicle();
}

State* RasWorld::Initialize() {
    return NULL;
}

State* RasWorld::GetCurretState() {
    return NULL;
}

bool RasWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {
    Simulation::step();
    auto targets = sim.perception();
    sim.controlEgoVehicle(targets);

}




