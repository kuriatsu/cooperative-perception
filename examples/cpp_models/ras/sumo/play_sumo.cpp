#include "play_sumo.h"

using namespace libtraci;


SumoSimulation::SumoSimulation() {
    return;
}

std::vector<std::string> SumoSimulation::perception() {

    // <id, distance>
    std::vector<std::string> targets;

    // get ego vehicle pose, route, current edge
    Pose ego_pose(Vehicle::getPosition(m_ego_name));
    ego_pose.theta = Vehicle::getAngle(m_ego_name);
    std::vector<std::string> ego_route = Vehicle::getRoute(m_ego_name);

    for (std::string& ped_id : Person::getIDList()) {
        std::string ped_edge = Person::getRoadID(ped_id);
        
        // get obstacles along to the ego vehicle route
        // if (std::find(ego_route.begin(), ego_route.end(), ped_id_edge) == ego_route.end()) continue;

        // get risk position
        Pose risk_pose(Person::getPosition(ped_id));
        Pose rel_risk_pose = risk_pose.transformTo(ego_pose);
        m_risks[ped_id].distance = rel_risk_pose.y;
        if (fabs(rel_risk_pose.x) < m_perception_range[0]/2 && 0 < rel_risk_pose.y && rel_risk_pose.y < m_perception_range[1]) {
            Person::setColor(ped_id, libsumo::TraCIColor(200, 0, 0));
            targets.emplace_back(ped_id);
        }
        else {
            Person::setColor(ped_id, libsumo::TraCIColor(0, 0, 200));
        }
    }

    return targets;
}


void SumoSimulation::controlEgoVehicle(const std::vector<std::string>& targets){

    std::vector<double> acc_list;
    double speed = Vehicle::getSpeed(m_ego_name);

	for (const std::string &it : targets) {
        Risk &target = m_risks[it];
        bool is_decel_target = false; 

        if (target.distance < 0) {
            is_decel_target = false;
        }
        else {
            is_decel_target = (target.risk == true);
        }

        if (!is_decel_target){
            continue;
        }

        double a = (pow(m_v_yield_speed, 2.0) - pow(speed, 2.0))/(2.0*(target.distance-m_safety_margin));
        acc_list.emplace_back(a);
    }

    auto a_itr = min_element(acc_list.begin(), acc_list.end());
    double acc = *a_itr;
    // int decel_target = target.distanceance(acc_list.begin(), a_itr);
    acc = (acc >= 0) ? std::min(acc, m_v_accel) : std::max(acc, -m_v_accel);
    speed += acc*1.0;
    std::cout << acc << std::endl;
    // if (speed <= m_v_yield_speed) {
    //     acc = 0.0;
    //  }
    // else if (speed >= m_v_max_speed) {
    //     acc = 0.0;
    // }
    
    Vehicle::setAccel(m_ego_name, acc);
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

void SumoSimulation::spawnPedestrians() {
    double interval = 1/m_density;
    std::cout << "test" << std::endl;

    // Generate random value
    std::mt19937 mt{std::random_device{}()};
    std::uniform_real_distribution<double> position_noise(-interval, interval), risk_val(0, 1);

    // add peds for each lane
    auto lane_list = Lane::getIDList();
    for (std::string& lane_id : lane_list) {

        const auto& allowed_list = Lane::getAllowed(lane_id);
        if (!std::count(allowed_list.begin(), allowed_list.end(),"pedestrian")) continue;

        std::string edge = Lane::getEdgeID(lane_id);
        double lane_length = Lane::getLength(lane_id);

        // add peds
        for (int i=0; i<lane_length; i+=(int)interval) {
            double position = i + position_noise(mt);
            std::string ped_id = lane_id + "-" + std::to_string(i);
            std::cout << position << ", " << lane_length << std::endl;
            Person::add(ped_id, edge, position);
            Person::setColor(ped_id, libsumo::TraCIColor(0, 0, 200));
            Person::appendWalkingStage(ped_id, {edge}, 0);
            Person::appendWaitingStage(ped_id, 1000);
            m_risks[ped_id] = Risk(ped_id, risk_val(mt)); 
        }
    }
}


int main(int argc, char* argv[]) {
    Simulation::start({"sumo-gui", "-c", "straight_random.sumocfg"});
    auto sim = SumoSimulation();
    sim.spawnPedestrians();
    sim.spawnEgoVehicle();

    for (int i = 0; i < 3000; i++) {
        Simulation::step();
        auto targets = sim.perception();
        sim.controlEgoVehicle(targets);
    }

    Simulation::close();
}
