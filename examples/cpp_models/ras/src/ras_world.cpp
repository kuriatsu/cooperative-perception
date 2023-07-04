#include "ras_world.h"

using namespace despot;

~World() {
    sim.close();
}

bool RasWorld::Connect(){
    sim.start();
}

State* RasWorld::Initialize() {
    sim.spawnPedestrians();
    sim.spawnEgoVehicle();
    RasState* state = new RasState();
    return NULL;
}

State* RasWorld::GetCurretState() {
    return NULL;
}

bool RasWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {


}




