#include "autoware_bridge/autonomous_driving.hpp"

#include <chrono>
#include <mutex>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

AutonomousDriving::AutonomousDriving(
  rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util,
  std::atomic<bool> & is_task_running)
: node_(node),
  autoware_bridge_util_(autoware_bridge_util),
  cancel_requested_(false),
  is_task_running_(is_task_running),
  state_(AutonomousDrivingTaskState::ENGAGE_AUTO_DRIVE),
  vehicle_motion_state_(MotionState::UNKNOWN)
{
  // Initialize autonomous client
  auto_drive_engage_client = node_->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/api/operation_mode/change_to_autonomous");
  operation_mode_state_sub_ = node_->create_subscription<OperationModeState>(
    "/api/operation_mode/state", 10,
    std::bind(&AutonomousDriving::operationModeStateCallback, this, std::placeholders::_1));
  vehicle_motion_state_sub_ = node_->create_subscription<MotionState>(
    "/api/motion/state", 10,
    std::bind(&AutonomousDriving::vehicleMotionStateCallback, this, std::placeholders::_1));
  route_state_sub_ = node_->create_subscription<RouteState>(
    "/api/routing/state", 10,
    std::bind(&AutonomousDriving::routeStateCallback, this, std::placeholders::_1));
}

void AutonomousDriving::execute(
  const std::string & task_id, const geometry_msgs::msg::PoseStamped & /*pose*/)
{
  autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "RUNNING");
  autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::TOTAL_RETRIES, "", 5);
  is_task_running_ = true;

  // Maximum number of initialization retries
  const int MAX_INIT_RETRIES = 5;
  int retry_counter = 0;
  bool success = false;

  while (true) {
    std::lock_guard<std::mutex> lock(task_mutex_);

    if (cancel_requested_.load()) {
      autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "CANCELLED");
      autoware_bridge_util_->updateTaskStatus(
        task_id, TaskRequestType::REASON, "Cancelled by user");
      RCLCPP_INFO(node_->get_logger(), "Localization task %s cancelled.", task_id.c_str());
      is_task_running_ = false;
      return;
    }

    if (retry_counter >= MAX_INIT_RETRIES) {
      // FAILURE

      // updateFailStatus(task_id, "Max retries elapsed");------>Implement it
      autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "FAILED");
      autoware_bridge_util_->updateTaskStatus(
        task_id, TaskRequestType::REASON, "Max retries elapsed");
      is_task_running_ = false;
      break;
    }

    if (success) {
      // SUCCESS
      autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "SUCCESS");
      is_task_running_ = false;
      break;
    }

    switch (state_) {
      case AutonomousDrivingTaskState::ENGAGE_AUTO_DRIVE:
        if (retry_counter == 1) {
          autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "RETRYING");
        }
        retry_counter++;
        autoware_bridge_util_->updateTaskStatus(
          task_id, TaskRequestType::RETRIES, "", retry_counter);
        engageAutoDrive();
        driving_start_time_ = node_->get_clock()->now();
        state_ = AutonomousDrivingTaskState::WAIT_AUTO_DRIVE_READY;
        break;

      case AutonomousDrivingTaskState::WAIT_AUTO_DRIVE_READY:
        if (operation_mode_state_.mode == OperationModeState::AUTONOMOUS) {
          // this->driving_stop_time_ = this->get_clock()->now();
          state_ = AutonomousDrivingTaskState::DRIVING;
        }
        // Timer for 10 second and retry
        else if (
          node_->get_clock()->now().seconds() - driving_start_time_.seconds() >
          DRIVE_WAIT_TIMEOUT_S) {
          RCLCPP_ERROR(node_->get_logger(), "Driving error, timeout expired");
          state_ = AutonomousDrivingTaskState::ENGAGE_AUTO_DRIVE;
        }
        break;

      case AutonomousDrivingTaskState::DRIVING:
        if (vehicle_motion_state_ == MotionState::STOPPED) {
          if (route_state_ == RouteState::ARRIVED) {
            success = true;
          } else {
            // check timer elapse Response : "HALT", 60sec
          }
        } else {
          // driving_stop_time_ = node_->get_clock()->now();
        }
        break;
      default:
        break;
    }
    // DRIVING -> HALT
    // 2nd attempt
    // UI -> CANCEL
    // UI -> RUN ROUTE PLANNING
  }
}

void AutonomousDriving::request_cancel()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  cancel_requested_ = true;
}

void AutonomousDriving::engageAutoDrive()
{
  /*auto_drive_engage_client is a ROS2 service client
    Its purpose is to send a request to a service(in this case,
    /api/operation_mode/change_to_autonomous) that instructs the vehicle or control system to
    switch to autonomous mode. service server is responsible for processing the request*/
  while (!auto_drive_engage_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        node_->get_logger(), "Interrupted while waiting for auto_drive_engage service. Exiting.");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "auto_drive_engage service not available, waiting again...");
  }
  // auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
  auto request = std::make_shared<ChangeOperationMode::Request>();

  auto future_result = auto_drive_engage_client->async_send_request(request);
}

void AutonomousDriving::operationModeStateCallback(const OperationModeState msg)
{
  operation_mode_state_ = msg;
}

void AutonomousDriving::vehicleMotionStateCallback(const MotionState msg)
{
  vehicle_motion_state_ = msg.state;
}

void AutonomousDriving::routeStateCallback(const RouteState msg)
{
  route_state_ = msg.state;
}