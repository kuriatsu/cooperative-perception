#include "task_allocation.h"
#include "ras_world.h"

class MyopicPolicy() {
protected:
    std::vector<std::string> history;
    RasWorld *world;

public:
    ACT_TYPE getAction(

