cmake_minimum_required(VERSION 2.8.3)
project(ras)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g3 -O0")
# set(CXXFLAGS "${CXXFLAGS} -g3")

find_library(libdespot NAMES despot PATHS ../../../build/ NO_DEFAULT_PATH)
find_package(Boost COMPONENTS program_options REQUIRED)

add_library(operator_model SHARED src/operator_model.cpp)
add_library(vehicle_model SHARED src/vehicle_model.cpp)
add_library(sumo_interface SHARED src/sumo_interface.cpp)
add_library(task_allocation SHARED src/task_allocation.cpp)
add_library(ras_world SHARED src/ras_world.cpp)
add_executable("${PROJECT_NAME}"
	src/main.cpp
	)

target_include_directories(operator_model PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include ../../../include)
target_include_directories(vehicle_model PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(sumo_interface PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include /usr/include/libsumo)
target_include_directories(task_allocation PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include ../../../include)
target_include_directories(ras_world PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include ../../../include json/include)
# target_include_directories("${PROJECT_NAME}" PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include ../../../include /usr/share/sumo/src)
target_include_directories("${PROJECT_NAME}" PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include ../../../include /usr/share/sumo/src ../../../include/json/include ${Boost_INCLUDE_DIRS})


target_link_libraries(task_allocation operator_model vehicle_model ${libdespot})
target_link_libraries(sumo_interface tracicpp)
target_link_libraries(operator_model ${libdespot})
target_link_libraries(ras_world task_allocation sumo_interface operator_model ${libdespot})
target_link_libraries("${PROJECT_NAME}"
        operator_model
        vehicle_model
        sumo_interface
        task_allocation
        ras_world
        ${libdespot}
        tracicpp
        ${Boost_LIBRARIES}
        )


