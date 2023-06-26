#include <iostream>
#include <libsumo/libtraci.h>

using namespace libtraci;

int main(int argc, char* argv[]) {
    Simulation::start({"sumo-gui", "-c", "straight_random.sumocfg"});
    auto edge_list = Edge::getIDList();
    auto route_list = Route::getIDList(); 
    Vehicle::add("ego_vehicle", route_list[0]);
    Vehicle::setColor("ego_vehicle", libsumo::TraCIColor(0, 200, 0));
    for (int i = 0; i < 3000; i++) {
        Simulation::step();
        Person::add("ped_"+std::to_string(i), "E0", 100);
        Person::setColor("ped_"+std::to_string(i), libsumo::TraCIColor(200, 0, 0));
        Person::appendWalkingStage("ped_"+std::to_string(i), {"E0", "-E0"}, 0);
        std::cout << Person::getPosition("ped_"+std::to_string(i)).getString() << std::endl;
    }

    Simulation::close();
}
