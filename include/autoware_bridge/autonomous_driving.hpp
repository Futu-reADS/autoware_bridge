#ifndef AUTONOMOUS_DRIVING_HPP
#define AUTONOMOUS_DRIVING_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"
#include "base_task.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/motion_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include "autoware_adapi_v1_msgs/srv/clear_route.hpp"


#include <atomic>
#include <memory>
#include <mutex>

enum class AutonomousDrivingTaskState { ENGAGE_AUTO_DRIVE, WAIT_AUTO_DRIVE_READY, DRIVING };

const double DRIVE_WAIT_TIMEOUT_S = 10.0;
const int MAX_DRIVE_RETRIES = 2;
const double MAX_EGO_HALT_TIME = 60.0;  // Halting time in seconds

class AutonomousDriving : public BaseTask
{
public:
  AutonomousDriving(
    rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util);
  void execute(const std::string & task_id, const geometry_msgs::msg::PoseStamped & pose)
    override;              // Executes Driving
  void cancel() override;  // Requests task cancellation

  // Alias for message types
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using MotionState = autoware_adapi_v1_msgs::msg::MotionState;
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
  using ClearRoute = autoware_adapi_v1_msgs::srv::ClearRoute;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util_;
  std::atomic<bool> is_cancel_requested_;
  AutonomousDrivingTaskState state_;
  OperationModeState operation_mode_state_;
  uint16_t vehicle_motion_state_;
  uint16_t route_state_;

  rclcpp::Time driving_start_time_;
  rclcpp::Time halt_start_time_;
  std::mutex task_mutex_;

  // Subscriber
  rclcpp::Subscription<OperationModeState>::SharedPtr operation_mode_state_sub_;
  rclcpp::Subscription<MotionState>::SharedPtr vehicle_motion_state_sub_;
  rclcpp::Subscription<RouteState>::SharedPtr route_state_sub_;

  // Client
  rclcpp::Client<ChangeOperationMode>::SharedPtr auto_drive_engage_client;
  rclcpp::Client<ClearRoute>::SharedPtr clear_route_client;

  // callbacks
  void vehicleMotionStateCallback(const MotionState msg);
  void routeStateCallback(const RouteState msg);
  void operationModeStateCallback(const OperationModeState msg);

  // Helper methods
  void engageAutoDrive();
  void cancelCurrentRoute();
};

#endif  // AUTONOMOUS_DRIVING_HPP
