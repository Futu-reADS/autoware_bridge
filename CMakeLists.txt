cmake_minimum_required(VERSION 3.8)
project(autoware_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(tier4_system_msgs REQUIRED)
find_package(ftd_master_msgs REQUIRED)
find_package(autoware_bridge_msgs REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(autoware_auto_system_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetTaskStatus.srv"
  DEPENDENCIES std_msgs
)

# Create a library target from common source files.
add_library(autoware_bridge_lib
  src/autoware_bridge_util.cpp
  src/localization.cpp
  src/route_planning.cpp
  src/autonomous_driving.cpp
)

  # Use target-specific include directories
  target_include_directories(autoware_bridge_lib PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

ament_target_dependencies(autoware_bridge_lib
  rclcpp
  std_msgs
  geometry_msgs
  autoware_adapi_v1_msgs
  tier4_system_msgs
  ftd_master_msgs
  autoware_bridge_msgs
  autoware_auto_msgs
  autoware_auto_system_msgs
  rosidl_default_runtime
)

# Link generated service interfaces to the library.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(autoware_bridge_lib ${cpp_typesupport_target})

# Create executable target for the node.
add_executable(autoware_bridge_node
  src/autoware_bridge.cpp
)
target_link_libraries(autoware_bridge_node
  autoware_bridge_lib
  ${cpp_typesupport_target}
)

# Install targets
install(TARGETS autoware_bridge_node autoware_bridge_lib
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

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  #find_package(ament_cmake_gtest REQUIRED)
  ##find_package(ament_lint_auto REQUIRED)
  ##ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_autoware_bridge_util.cpp
  )
  if(TARGET test_${PROJECT_NAME})
    ament_target_dependencies(test_${PROJECT_NAME}
      rclcpp
      std_msgs
      geometry_msgs
      autoware_bridge_msgs
      ftd_master_msgs
      autoware_adapi_v1_msgs
    )
    target_link_libraries(test_${PROJECT_NAME} autoware_bridge_lib)
  endif()
endif()

ament_package()
