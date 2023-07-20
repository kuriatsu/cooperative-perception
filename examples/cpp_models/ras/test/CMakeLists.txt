cmake_minimum_required(VERSION 2.8.3)
project(text_task_allocation)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g3 -O0")
set(CXXFLAGS "${CXXFLAGS} -g3")

find_library(libdespot NAMES despot PATHS ../../../../build/ NO_DEFAULT_PATH)
add_executable("${PROJECT_NAME}"
	test_task_allocation.cpp
	)

target_include_directories("${PROJECT_NAME}" PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../include ../../../../include)


target_link_libraries("${PROJECT_NAME}"
        ${libdespot}
        )

