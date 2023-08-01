cmake_minimum_required(VERSION 2.8.3)
project(test_ta)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g3 -O0")
set(CXXFLAGS "${CXXFLAGS} -g3")

find_library(libdespot NAMES despot PATHS ../../../../build/ NO_DEFAULT_PATH)
add_executable("${PROJECT_NAME}"
	test_task_allocation.cpp
	)
add_library(task_allocation SHARED ../src/task_allocation.cpp)
add_library(operator_model SHARED ../src/operator_model.cpp)
add_library(vehicle_model SHARED ../src/vehicle_model.cpp)

target_include_directories("${PROJECT_NAME}" PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../include ../../../../include)
target_include_directories(task_allocation PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../include ../../../../include)
target_include_directories(operator_model PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../include ../../../../include)
target_include_directories(vehicle_model PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../include)

target_link_libraries(operator_model ${libdespot})
target_link_libraries(task_allocation operator_model vehicle_model ${libdespot})
target_link_libraries("${PROJECT_NAME}"
        ${libdespot}
        task_allocation
        )

