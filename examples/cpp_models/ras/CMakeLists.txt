cmake_minimum_required(VERSION 2.8.3)

add_executable("${PROJECT_NAME}_ras"
	src/ras.cpp
	src/main.cpp
	src/operator_model.cpp
	)

target_link_libraries("${PROJECT_NAME}_ras"
	"${PROJECT_NAME}"
	)

install(TARGETS "${PROJECT_NAME}_ras"
	RUNTIME DESTINATION "{BINARY_INSTALL_PATH}"
	)

