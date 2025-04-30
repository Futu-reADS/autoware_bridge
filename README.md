**Repository working as a bridge between UI_bridge and Autoware**


## About

`autoware_bridge` is a ROS 2 node that orchestrates task execution between a user interface (`UI_bridge`) and 
the core Autoware modules (localization, route planning, autonomous driving). 
It handles:
- Task request subscriptions
- Asynchronous task execution
- Cancellation requests
- Status queries via ROS 2 service 


## Features
---------------------------------------------------------------------------------
- **Localization**, **Route Planning**, and **Autonomous Driving** task handling
- Task rejection if another task is already running
- Publish task status and cancellation responses
- Automatic reinitialization requests when localization quality degrades
- ROS 2 service to query current task status

---------------------------------------------------------------------------------

## Requirements
---------------------------------------------------------------------------------
- ROS 2 Humble (or later)
- C++
- Dependencies listed in [`package.xml`](package.xml):
  - `rclcpp`, `std_msgs`, `geometry_msgs`, `diagnostic_msgs`, `ftd_master_msgs`, `tier4_system_msgs`
  - `ament_cmake`, `ament_lint_auto`, etc.

---------------------------------------------------------------------------------

## Installation & Build
---------------------------------------------------------------------------------
```bash
# Clone standalone
git clone git@github.com:Futu-reADS/autoware_bridge.git

#clone via autoware.repos 
-Please make entry in autoware.repos present inside autoware.FTD in proper format (following others)
-vcs import src < autoware.repos --recursive
-vcs pull 

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select autoware_bridge --cmake-clean-cache
---------------------------------------------------------------------------------

## Usages
---------------------------------------------------------------------------------
- Source your workspace:
  source ~/your_ws/install/setup.bash
- Launch the bridge node standalone 
  ros2 run autoware_bridge autoware_bridge_node
- Monitor responses:
  ros2 topic echo /autoware_bridge/task_response
  ros2 topic echo /autoware_bridge/cancel_task_response
  ros2 topic echo /autoware_bridge/reinitialize 
  
- Query status via service:
  ros2 service call /check_task_status autoware_bridge/srv/GetTaskStatus "{ task_id: 'localization_xx' }"

---------------------------------------------------------------------------------

## Testing
---------------------------------------------------------------------------------
This package includes a comprehensive ament_gtest suite under test/. To run all tests:
# From the root of your workspace:
  colcon build --packages-select autoware_bridge --cmake-clean-cache
  colcon test --packages-select autoware_bridge
  colcon test-result --verbose
  
  Code coverage is also integrated with this package and For code coverage you can use below command 
  then one url will open and will show the line coverage
  
  lcov --capture --directory build/autoware_bridge --output-file coverage.info
  lcov --remove coverage.info '/usr/*' '*/test/*' --output-file coverage_filtered.info
  lcov --extract coverage_filtered.info '*/autoware_bridge/src/*.cpp' --output-file autoware_bridge_src.info
  genhtml autoware_bridge_src.info --output-directory autoware_bridge_src_report
  xdg-open autoware_bridge_src_report/index.html 

Recomended: run one class test at a time by commenting out all other classes in CMakeLists.txt
---------------------------------------------------------------------------------

##Contributing
---------------------------------------------------------------------------------
-Fork this repository
-Create a feature branch
-Commit and push your changes
-Open a Pull Request

Please follow the existing code style and add new tests for any behavior you introduce.
---------------------------------------------------------------------------------

License
This project is licensed under the Apache License 2.0.


