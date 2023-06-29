#include "play_sumo.h"
#include "libgeometry.h"

using namespace libtraci;


void SumoSimulation::perception(const std::string ego_name) {

    // <id, distance>
    std::unordered_map<std::string, double> targets;

    // get ego vehicle pose, route, current edge
    Pose ego_pose(Vehicle::GetPosition(ego_name));
    ego_pose.theta = Vehicle::GetAngle(ego_name);
    std::vector<std::string> ego_route = Vehicle::GetRoute(ego_name);
    std::string ego_edge = Vehicle::GetRoadID(ego_name);

    for (std::string& ped : Edge::getLastStepPersonIDs(edge)) {
        std::string ped_edge = Person::getRoadID(ped);
        
        // get obstacles along to the ego vehicle route
        // if (std::find(ego_route.begin(), ego_route.end(), ped_edge) == ego_route.end()) continue;

        // get risk position
        Pose risk_pose(Person::getPosition(ped));
        Pose rel_risk_pose = risk_pose.transformTo(ego_pose);
        if (fabs(rel_risk_pose.x) < perception_range[0]/2 && 0 < rel_risk_pose.y && rel_risk_pose.y < perception_range[1]) {
            targets[ped] = rel_risk_pose.y;
        }
    }

    return targets;
}


void SumoSimulation::controlEgoVehicle(const std::string ego_name, const auto& targets){

    vector<double> acc_list;
    double speed = Vehicle::getSpeed(ego_name);

	for (const auto &it : targets) {
        double &dist = it.second();
        bool is_decel_target = false; 

        if (dist < 0) {
            is_decel_target = false;
        }
        else {
            is_decel_target = (*it == true);
        }

        if (!is_decel_target){
            continue;
        }

        double a = (pow(yield_speed, 2.0) - pow(speed, 2.0))/(2.0*(dist-safety_margin));
        acc_list.emplace_back(a);
    }

    auto a_itr = min_element(acc_list.begin(), acc_list.end());
    double acc = *a_itr;
    // int decel_target = distance(acc_list.begin(), a_itr);
    a = (a >= 0) ? min(acc, m_v_accel) : max(acc, -m_v_accel);
    speed += a*delta_t;
    if (speed <= yield_speed) {
        speed =	yield_speed;
        a = 0.0;
    }
    else if (speed >= ideal_speed) {
        speed = ideal_speed;
        a = 0.0;
    }
    
    Vehicle::setAccel(ego_name, a);
}


void SumoSimulation::spawnEgoVehicle() {
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

void SumoSimulation::spawnPedestrians(const int density) {
    double interval = 1/density;

    // Generate random value
    std::mt19937 mt{std::random_device{}()};
    std::uniform_real_distribution<double> position_noise(-interval, interval), risk_val(0, 1);

    // add peds for each lane
    auto lane_list = Lane::getIDList();
    for (std::string& lane_id : lane_list) {
        std::string edge = Lane::getEdgeID(lane_id);
        double length = Lane::getLength(lane_id)

        // add peds
        for (int i=0; i<length; i+=(int)interval) {
            double position = i + position_noise(mt);
            Person::add(std::to_string(i), edge, position)
            Person::setColor(std::to_string(i), libsumo::TraCIColor(200, 0, 0));
            // Person::appendWalkingStage(std::to_string(i), {edge}, 0);
            Person::appendWaitingState(std::to_string(i), 1000);
            risks[std::to_string(i)] = risk_val(mt); 
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
