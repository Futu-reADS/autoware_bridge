#include "autoware_bridge/autonomous_driving.hpp"

#include <chrono>

using namespace std::chrono_literals;

AutonomousDriving::AutonomousDriving(
  rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util,
  std::atomic<bool> & is_task_running)
: node_(node),
  autoware_bridge_util_(autoware_bridge_util),
  is_cancel_requested_(false),
  is_task_running_(is_task_running),
  state_(AutonomousDrivingTaskState::ENGAGE_AUTO_DRIVE),
  operation_mode_state_(),
  vehicle_motion_state_(MotionState::UNKNOWN),
  route_state_(RouteState::UNKNOWN),
  driving_start_time_(rclcpp::Time(0)),
  halt_start_time_(rclcpp::Time(0))
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
  autoware_bridge_util_->updateRunningStatus(task_id, 5);
  is_task_running_ = true;

  // Maximum number of drive retries
  int retry_counter = 0;
  bool success = false;
  auto max_operation_mode_wait_time =
    30.0;                                // Timeout for waiting for autonomous mode to be engaged
  auto max_vehicle_stopped_time = 60.0;  // Timeout for vehicle being stopped for too long

  rclcpp::Time operation_mode_wait_start_time = node_->get_clock()->now();

  while (true) {
    std::lock_guard<std::mutex> lock(task_mutex_);

    if (is_cancel_requested_) {
      // CANCEL
      autoware_bridge_util_->updateCancellationStatus(task_id, "Cancelled by user");
      RCLCPP_INFO(node_->get_logger(), "Localization task %s cancelled.", task_id.c_str());
      is_task_running_ = false;
      return;
    }

    if (retry_counter >= MAX_DRIVE_RETRIES) {
      // FAILURE
      autoware_bridge_util_->updateFailStatus(task_id, "Max retries elapsed");
      is_task_running_ = false;
      break;
    }

    if (success) {
      // SUCCESS
      autoware_bridge_util_->updateSuccessStatus(task_id);
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
          state_ = AutonomousDrivingTaskState::DRIVING;
        }
        // Timeout for waiting for autonomous mode
        else if (
          node_->get_clock()->now().seconds() - operation_mode_wait_start_time.seconds() >
          max_operation_mode_wait_time) {
          autoware_bridge_util_->updateFailStatus(task_id, "Timeout waiting for autonomous mode");
          is_task_running_ = false;
          break;
        }
        // Timer for 10 seconds and retry if the mode is not autonomous
        else if (
          node_->get_clock()->now().seconds() - driving_start_time_.seconds() >
          DRIVE_WAIT_TIMEOUT_S) {
          RCLCPP_ERROR(node_->get_logger(), "Driving error, timeout expired");
          state_ = AutonomousDrivingTaskState::ENGAGE_AUTO_DRIVE;
        }
        break;

      case AutonomousDrivingTaskState::DRIVING:
        if (vehicle_motion_state_ == MotionState::STOPPED) {
          // Check if the vehicle has been stopped for too long (timeout)
          if (
            halt_start_time_.seconds() > 0 &&
            node_->get_clock()->now().seconds() - halt_start_time_.seconds() >
              max_vehicle_stopped_time) {
            RCLCPP_WARN(node_->get_logger(), "HALT: 60 seconds elapsed while stopped.");
            // Handle HALT logic: re-engage, retry, or reset
            state_ = AutonomousDrivingTaskState::ENGAGE_AUTO_DRIVE;
          }
        } else {
          // Vehicle is moving again, so start/reset the halt timer
          halt_start_time_ = node_->get_clock()->now();  // Start timer when driving starts
        }

        // If the route state is arrived, we are done
        if (route_state_ == RouteState::ARRIVED) {
          success = true;
          retry_counter = 0;
        }
        break;

      default:
        break;
    }
  }
}

void AutonomousDriving::cancelRequested()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  is_cancel_requested_ = true;
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
  auto request = std::make_shared<ChangeOperationMode::Request>();
  auto future_result = auto_drive_engage_client->async_send_request(request);

  // It's good practice to add a maximum retry limit in case the service is not available.
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
