cmake_minimum_required(VERSION 3.8)
project(cooperative_perception)

add_compile_options(-g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} $ENV{DESPOT_ROOT}/build)
message(STATUS $ENV{DESPOT_ROOT})
#find_package(Despot REQUIRED)
find_package(Despot CONFIG REQUIRED PATHS $ENV{DESPOT_ROOT})

include_directories (
    include
    ${DESPOT_INCLUDE_DIRS}
)
link_directories (
    ${DESPOT_LIBRARIES}
    ${DESPOT_ROOT}/../../
)
message(${DESPOT_INCLUDE_DIRS})
# target_include_directories(${PROJECT_NAME} PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

############################
# msg
############################
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Intervention.msg"
    "msg/PredictedObject.msg"
    "srv/UpdatePerception.srv"
    "srv/Intervention.srv"
    "srv/State.srv"
    DEPENDENCIES
    "std_msgs"
    "unique_identifier_msgs"
    ADD_LINTER_TESTS
) 
ament_export_dependencies(rosidl_default_runtime)

############################
# cooperative_perception
############################

add_executable(${PROJECT_NAME}_node src/cooperative_perception.cpp src/cp_pomdp.cpp src/cp_world.cpp src/operator_model.cpp src/vehicle_model.cpp src/modelbase_planner.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  autoware_auto_perception_msgs
  autoware_auto_planning_msgs
  geometry_msgs
  unique_identifier_msgs
)
target_link_libraries(${PROJECT_NAME}_node despot)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

## ros galactic or lower
# rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
## ros humble
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")

############################
# cp_ros_interface
############################

add_executable(cp_ros_interface_node src/cp_ros_interface.cpp)
ament_target_dependencies(cp_ros_interface_node
  rclcpp
  autoware_auto_perception_msgs
  autoware_auto_planning_msgs
  geometry_msgs
  unique_identifier_msgs
)

## to use Intervention.msg
# rosidl_target_interfaces(cp_ros_interface_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(cp_ros_interface_node "${cpp_typesupport_target}")


install(TARGETS cp_ros_interface_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
