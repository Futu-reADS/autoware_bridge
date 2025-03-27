#include "autoware_bridge/localization.hpp"

#include <chrono>

using namespace std::chrono_literals;
Localization::Localization(
  rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util)
: node_(node),
  autoware_bridge_util_(autoware_bridge_util),
  is_cancel_requested_(false),
  state_(LocalizationTaskState::INITIALIZATION),
  localization_state_(LocalizationInitializationState::UNKNOWN),
  localization_quality_(false),
  localization_start_time_(rclcpp::Time(0))
{
  init_pose_publisher_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("/initialpose", 10);

  localization_state_subscriber_ = node_->create_subscription<LocalizationInitializationState>(
    "/api/localization/initialization_state", 10,
    std::bind(&Localization::localizationStateCallback, this, std::placeholders::_1));

  localization_quality_subscriber_ = node_->create_subscription<ModeChangeAvailable>(
    "/system/component_state_monitor/component/autonomous/localization",
    rclcpp::QoS(1).transient_local(),
    std::bind(&Localization::localizationQualityCallback, this, std::placeholders::_1));
}

void Localization::execute(
  const std::string & task_id, const geometry_msgs::msg::PoseStamped & init_pose)
{
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
      return;
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
      RCLCPP_INFO(node_->get_logger(), "Localization task %s successful.", task_id.c_str());
      break;
    }

    switch (state_) {
        // sendCmdGate(); // check this whether required or not

      case LocalizationTaskState::INITIALIZATION:
        if (retry_counter == 1) {
          autoware_bridge_util_->updateTaskStatus(task_id, "RETRYING");
        }

        if (retry_counter <= MAX_INIT_RETRIES) {
          pubInitPose(init_pose);
          std::this_thread::sleep_for(500ms);
          state_ = LocalizationTaskState::LOCALIZATION;
          autoware_bridge_util_->updateTaskRetries(task_id, retry_counter);
          retry_counter++;
        } else {
          timeout = true;
        }

        break;
      case LocalizationTaskState::LOCALIZATION:

        switch (localization_state_) {
          case LocalizationInitializationState::UNINITIALIZED:
            state_ = LocalizationTaskState::INITIALIZATION;
            break;

          case LocalizationInitializationState::INITIALIZED:
            localization_start_time_ = node_->get_clock()->now();
            state_ = LocalizationTaskState::LOCALIZATION_CHECK;
            break;
          default:
            RCLCPP_INFO(node_->get_logger(), "Localization state: %d", localization_state_);
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
          } else {
            RCLCPP_INFO(node_->get_logger(), "Localisation quality is poor, retrying localization");
            state_ = LocalizationTaskState::INITIALIZATION;
          }
        } else {
          RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000, "Waiting for localization result");
        }
        break;
      default:
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
}

void Localization::localizationStateCallback(const LocalizationInitializationState & msg)
{
  localization_state_ = msg.state;
}
// check it's usability
// void Localization::sendCmdGate()
// {
//   RCLCPP_INFO(node_->get_logger(), "Sending command to gate...");
// }

void Localization::pubInitPose(const geometry_msgs::msg::PoseStamped & init_pose)
{
  init_pose_publisher_->publish(init_pose);
}

bool Localization::getLocalizationQuality() const
{
  return localization_quality_;
}