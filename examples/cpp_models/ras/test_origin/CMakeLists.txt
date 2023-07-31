cmake_minimum_required(VERSION 2.8.3)
project(test_ras)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g3 -O0")
set(CXXFLAGS "${CXXFLAGS} -g3")

find_library(libdespot NAMES despot PATHS ../../../../build/ NO_DEFAULT_PATH)
add_executable("${PROJECT_NAME}"
	main.cpp
	)
add_library(ras SHARED ../ras.cpp)
add_library(operator_model SHARED ../operator_model.cpp)

target_include_directories("${PROJECT_NAME}" PUBLIC  ../../../../include)
target_include_directories(ras PUBLIC  ../../../../include)

target_link_libraries(ras operator_model ${libdespot})
target_link_libraries("${PROJECT_NAME}"
        ${libdespot}
        operator_model
        ras
        )

