#pragma once

#include <iostream>
#include <libsumo/libtraci.h>


using namespace libtraci;

class PlaySumo {
public:
    void spawnEgoVehicle();
    void spawnPedestrians();
    void controlEgoVehicle();
    void perception();


