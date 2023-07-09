cmake_minimum_required(VERSION 2.8.3)
project(ras)

add_library(operator_model SHARED src/operator_model.cpp)
add_library(vehicle_model SHARED src/vehicle_model.cpp)
add_library(sumo_interface SHARED src/sumo_interface.cpp)
add_library(task_allocation SHARED src/task_allocation.cpp)
add_library(ras_world SHARED src/ras_world.cpp)

target_include_directories(operator_model PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(vehicle_model PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(sumo_interface PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(task_allocation PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(ras_world PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

include_directories(../../../include)
link_directories(../../../build)

target_link_libraries(task_allocation operator_model vehicle_model)

add_executable("${PROJECT_NAME}"
	src/main.cpp
	)


target_link_libraries("${PROJECT_NAME}"
        operator_model
        vehicle_model
        sumo_interface
        task_allocation
        ras_world
	)

target_include_directories("${PROJECT_NAME}" PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

