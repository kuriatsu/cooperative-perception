cmake_minimum_required(VERSION 2.8.3)
project(despot)

add_executable("${PROJECT_NAME}_ras"
	main.cpp
	ras.cpp
    ras.h
	operator_model.cpp
    operator_model.h
	)

target_link_libraries("${PROJECT_NAME}_ras"
	"${PROJECT_NAME}"
	)

install(TARGETS "${PROJECT_NAME}_ras"
	RUNTIME DESTINATION "{BINARY_INSTALL_PATH}"
	)

