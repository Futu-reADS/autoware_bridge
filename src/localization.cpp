#include "autoware_bridge/localization.hpp"

#include <chrono>

using namespace std::chrono_literals;
Localization::Localization(
  std::shared_ptr<rclcpp::Node> node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util)
: node_(node),
  autoware_bridge_util_(autoware_bridge_util),
  is_cancel_requested_(false),
  state_(LocalizationTaskState::INITIALIZATION),
  localization_state_(LocalizationInitializationState::UNKNOWN),
  localization_quality_(false),
  localization_start_time_(rclcpp::Time(0))
{
  init_pose_publisher_ =
    node_->create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);

  localization_state_subscriber_ = node_->create_subscription<LocalizationInitializationState>(
    "/api/localization/initialization_state", rclcpp::QoS(1).transient_local(),
    std::bind(&Localization::localizationStateCallback, this, std::placeholders::_1));

  localization_quality_subscriber_ = node_->create_subscription<ModeChangeAvailable>(
    "/system/component_state_monitor/component/autonomous/localization",
    rclcpp::QoS(1).transient_local(),
    std::bind(&Localization::localizationQualityCallback, this, std::placeholders::_1));

    current_gate_mode_publisher_ = node_->create_publisher<tier4_control_msgs::msg::GateMode>(
    "/input/current_gate_mode", rclcpp::QoS{1});

    RCLCPP_INFO(node_->get_logger(), " [AVI00] Localization constructor called successfully.");
}

void Localization::execute(
  const std::string & task_id, const geometry_msgs::msg::PoseStamped & init_pose)
{
  RCLCPP_INFO(node_->get_logger(), " [AVI0] execute function entry.");
  autoware_bridge_util_->updateTaskStatus(task_id, "RUNNING");
  // SET FOR MAX_RETRIES

  // Maximum number of initialization retries
  int retry_counter = 0;
  bool success = false;
  bool timeout = false;

  while (true) {
    std::lock_guard<std::mutex> lock(task_mutex_);

    if (is_cancel_requested_) {
      // CANCEL
      autoware_bridge_util_->updateTaskStatus(task_id, "CANCELLED");
      RCLCPP_INFO(node_->get_logger(), "Localization task %s cancelled.", task_id.c_str());
      break;
    }

    if (timeout) {
      // TIMEOUT
      autoware_bridge_util_->updateTaskStatus(task_id, "TIMEOUT");
      RCLCPP_ERROR(node_->get_logger(), "Localization task %s timeout.", task_id.c_str());
      break;
    }

    if (success) {
      // SUCCESS
      autoware_bridge_util_->updateTaskStatus(task_id, "SUCCESS");
      RCLCPP_INFO(node_->get_logger(), "[DEEP]100 Localization task %s successful.", task_id.c_str());
      break;
    }

    switch (state_) {
        // sendCmdGate(); // check this whether required or not

      case LocalizationTaskState::INITIALIZATION:
        if (retry_counter == 1) {
          autoware_bridge_util_->updateTaskStatus(task_id, "RETRYING");
          RCLCPP_INFO(node_->get_logger(), "RETRYING [AVI1] state: %d retry_counter %d", static_cast<int>(state_), retry_counter);
        }

        if (retry_counter <= MAX_INIT_RETRIES) {
          pubInitPose(init_pose);
          std::this_thread::sleep_for(500ms);
          state_ = LocalizationTaskState::LOCALIZATION;
          autoware_bridge_util_->updateTaskRetries(task_id, retry_counter);
          retry_counter++;
          RCLCPP_INFO(node_->get_logger(), "[AVI2] state: %d retry_counter %d ", static_cast<int>(state_),retry_counter);
        } else {
          timeout = true;
          RCLCPP_INFO(node_->get_logger(), "[AVI3] state: %d retry_counter %d ", static_cast<int>(state_),retry_counter);
        }

        break;
      case LocalizationTaskState::LOCALIZATION:

        switch (localization_state_) {
          case LocalizationInitializationState::UNINITIALIZED:
            sendCmdGate();
            state_ = LocalizationTaskState::INITIALIZATION;
            RCLCPP_INFO(node_->get_logger(), "[AVI4] Localization state: %d", localization_state_);
            break;

          case LocalizationInitializationState::INITIALIZED:
            localization_start_time_ = node_->get_clock()->now();
            state_ = LocalizationTaskState::LOCALIZATION_CHECK;
            RCLCPP_INFO(node_->get_logger(), "[AVI5] Localization state: %d", localization_state_);
            break;
          default:
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "[AVI6] Localization state: %d", localization_state_);
            // This may lead to busy loop , is there any other thing which we can do here?
            break;
        }
        break;
      case LocalizationTaskState::LOCALIZATION_CHECK:
        if (
          node_->get_clock()->now().seconds() - localization_start_time_.seconds() >
          LOC_WAIT_TIMEOUT_S) {
          if (localization_quality_) {
            success = true;
            RCLCPP_INFO(node_->get_logger(), "[AVI7] Localization state: %d", localization_state_);
          } else {
            RCLCPP_INFO(node_->get_logger(), "[AVI8] Localisation quality is poor, retrying localization");
            state_ = LocalizationTaskState::INITIALIZATION;
          }
        } else {
          RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000, "[AVI9]  Waiting for localization result");
        }
        break;
      default:
        RCLCPP_ERROR(node_->get_logger(), "[AVI10] Unknown state: %d", static_cast<int>(state_));
        break;
    }
  }

  // std::this_thread::sleep_for(100ms);
}
void Localization::cancel()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  is_cancel_requested_ = true;
}

// make it a callback to update the status

void Localization::localizationQualityCallback(const ModeChangeAvailable & msg)
{
  localization_quality_ = msg.available;
  RCLCPP_INFO(node_->get_logger(), "[AVI11] localizationQualityCallback: localization_quality_%d", localization_quality_);
}

void Localization::localizationStateCallback(const LocalizationInitializationState & msg)
{
  localization_state_ = msg.state;
  RCLCPP_INFO(node_->get_logger(), "[AVI12] localizationStateCallback:localization_state_ %d", localization_state_);
  //uint16 UNKNOWN = 0, UNINITIALIZED = 1, INITIALIZING = 2, INITIALIZED = 3
}
// check it's usability
// void Localization::sendCmdGate()
// {
//   RCLCPP_INFO(node_->get_logger(), "Sending command to gate...");
// }

void Localization::pubInitPose(const geometry_msgs::msg::PoseStamped & init_pose)
{
  geometry_msgs::msg::PoseWithCovarianceStamped target_pose;
  
  // Copy header from input pose.
  target_pose.header = init_pose.header;
  
  // frame_id and stamp.
  target_pose.header.frame_id = "map";
  target_pose.header.stamp = node_->now();
  
  // Convert PoseStamped to PoseWithCovarianceStamped by copying the pose.
  target_pose.pose.pose = init_pose.pose;
  
  RCLCPP_INFO(
    node_->get_logger(),
    "target_pose: frame_id=%s, stamp=%u.%u, position=(%f, %f, %f), orientation=(%f, %f, %f, %f)",
    target_pose.header.frame_id.c_str(),
    target_pose.header.stamp.sec,
    target_pose.header.stamp.nanosec,
    target_pose.pose.pose.position.x,
    target_pose.pose.pose.position.y,
    target_pose.pose.pose.position.z,
    target_pose.pose.pose.orientation.x,
    target_pose.pose.pose.orientation.y,
    target_pose.pose.pose.orientation.z,
    target_pose.pose.pose.orientation.w);
  
  init_pose_publisher_->publish(target_pose);
}

void Localization::sendCmdGate()
{
  tier4_control_msgs::msg::GateMode mode_msg;
  mode_msg.data = tier4_control_msgs::msg::GateMode::AUTO;
  current_gate_mode_publisher_->publish(mode_msg);
  RCLCPP_INFO(node_->get_logger(), "Sending cmd_gate");

}