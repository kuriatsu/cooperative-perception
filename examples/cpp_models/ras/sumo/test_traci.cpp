#include <iostream>
#include <libsumo/libtraci.h>

using namespace libtraci;

int main(int argc, char* argv[]) {
    Simulation::start({"sumo-gui", "-c", "straight_random.sumocfg"});
    for (int i = 0; i < 3000; i++) {
        Simulation::step();
        Person::add("ped_"+std::to_string(i), "E0", 0);
        Person::setColor("ped_"+std::to_string(i), libsumo::TraCIColor(200, 0, 0));
        Person::appendWalkingStage("ped_"+std::to_string(i), {"E0", "-E0"}, 0);
        std::cout << Person::getPosition("ped_"+std::to_string(i)).getString() << std::endl;
    }

    Simulation::close();
}
