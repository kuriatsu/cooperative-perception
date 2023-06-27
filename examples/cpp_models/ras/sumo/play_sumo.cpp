#include "play_sumo.h"

using namespace libtraci;

void PlaySumo::perception(const std::string ego_name) {

    auto edges = Vehicle::GetRoute(ego_name);
    for (auto& edge : edges) {
        auto peds = Edge::getLastStepPersonIDs(edge);

void PlaySumo::controlEgoVehicle(const std::string ego_name) {


void PlaySumo::spawnEgoVehicle() {
    auto edge_list = Edge::getIDList();
    auto route_list = Route::getIDList(); 
    if (Route::getIDList().empty()) {
        Route::add("ego_vehicle_route", {"E0"});
        std::cout << "add new route: " << Route::getIDList()[0] << std::endl;
    }
    
    // for (auto &itr : Route::getIDList()){
    //     std::cout << itr << std::endl;
    // }

    Vehicle::add("ego_vehicle", Route::getIDList()[0]);
    Vehicle::setColor("ego_vehicle", libsumo::TraCIColor(0, 200, 0));
    Vehicle::setMaxSpeed("ego_vehicle", 50.0/3.6);
    Vehicle::setAccel("ego_vehicle", 1.5*9.8);
}

void PlaySumo::spawnPedestrians(const int density) {
    double interval = 1/density;
    std::mt19937 mt{std::random_device{}()};
    std::uniform_real_distribution<double> dist(-interval, interval);

    auto lane_list = Lane::getIDList();
    for (std::string& lane_id : lane_list) {
        std::string edge = Lane::getEdgeID(lane_id);
        double length = Lane::getLength(lane_id)
        for (int i=0; i<length; i+=(int)interval) {
            double position = i + dist(mt);
            Person::add(lane_id+"_ped_"+std::to_string(i), edge, 0);
            Person::setColor(lane_id+"_ped_"+std::to_string(i), libsumo::TraCIColor(200, 0, 0));
            // Person::appendWalkingStage(lane_id+"_ped_"+std::to_string(i), {edge}, 0);
            Person::appendWaitingState(lane_id+"_ped_"+std::to_string(i), 1000);
        }
    }
}


int main(int argc, char* argv[]) {
    Simulation::start({"sumo-gui", "-c", "straight_random.sumocfg"});
    for (int i = 0; i < 3000; i++) {
        Simulation::step();
    }

    Simulation::close();
}
