#include <iostream>
#include <libsumo/libtraci.h>

using namespace libtraci;

int main(int argc, char* argv[]) {
    Simulation::start({"sumo", "-c", "straight_random.sumocfg"});
    for (int i = 0; i < 3000; i++) {
        Simulation::step();
    }
    Simulation::close();
}
