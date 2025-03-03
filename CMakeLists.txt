cmake_minimum_required(VERSION 3.8)
project(autoware_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(tier4_system_msgs REQUIRED)
find_package(ftd_master_msgs REQUIRED)
find_package(autoware_bridge_msgs REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(autoware_auto_mapping_msgs REQUIRED)
find_package(autoware_auto_system_msgs REQUIRED)
find_package(autoware_auto_geometry_msgs REQUIRED)
find_package(autoware_auto_common REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(
  include
)

# Service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetTaskStatus.srv"
  DEPENDENCIES std_msgs
)

# Add executable
add_executable(autoware_bridge_node
  src/autoware_bridge.cpp
  src/autoware_bridge_util.cpp
  src/localization.cpp
  src/route_planning.cpp
  src/autonomous_driving.cpp
)

# Link dependencies
ament_target_dependencies(autoware_bridge_node
  rclcpp
  std_msgs
  autoware_adapi_v1_msgs
  tier4_system_msgs
  ftd_master_msgs
  autoware_bridge_msgs
  autoware_auto_msgs
  autoware_auto_vehicle_msgs
  autoware_auto_control_msgs
  autoware_auto_planning_msgs
  autoware_auto_perception_msgs
  autoware_auto_mapping_msgs
  autoware_auto_system_msgs
  autoware_auto_geometry_msgs
  autoware_auto_common
  rosidl_default_runtime
)

# Link generated service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(autoware_bridge_node ${cpp_typesupport_target})

# Install targets
install(TARGETS autoware_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install service files
install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
