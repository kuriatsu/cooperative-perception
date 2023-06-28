#include "play_sumo.h"

using namespace libtraci;

std::vector<double> PlaySumo::getRelativePosition(const std::vectol<double>& ego_pose, const double& target_position) {
    double& x = target_position[0];
    double& y = target_position[1];
    double& theta = ego_pose[2];
    s_x = x - ego_pose[0];
    s_y = y - ego_pose[1];
    r_x = s_x * cos(theta) - s_y * sin(theta); 
    r_y = s_x * sin(theta) + s_y * cos(theta); 
    return {r_x, r_y}; 
}

void PlaySumo::perception(const std::string ego_name) {

    std::vector<double> perception_range = {50, 150};
    std::vector<double> ego_pose = Vehicle::GetLanePosition(ego_name);
    ego_pose.emplace_back(Vehicle::GetAngle(ego_name));
    std::vector<std::string> ego_route = Vehicle::GetRoute(ego_name);
    std::string ego_edge = Vehicle::GetRoadID(ego_name);

    for (std::string& ped : Edge::getLastStepPersonIDs(edge)) {
        std::string ped_edge = Person::getRoadID(ped);
        
        // get obstacles along to the ego vehicle route
        if (std::find(ego_route.begin(), ego_route.end(), ped_edge) == ego_route.end()) continue;

        auto ped_position = Person::getPosition(ped);
        auto r_ped_position = getRelativePosition(ego_pose, ped_position);
        if (-perception_range[0]/2 < r_ped_position[0]
            && r_ped_position[0] < perception_range[0]/2
            && 0 < r_ped_position[1]
            && r_ped_position[1] < perception_range[1]) {
            target_list.append(


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
