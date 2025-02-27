#include "autoware_bridge/localization.hpp"

#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
Localization::Localization(
  rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util,
  std::atomic<bool> & is_task_running)
: node_(node),
  autoware_bridge_util_(std::move(autoware_bridge_util)),  // Move shared_ptr for efficiency
  cancel_requested_(false),
  is_task_running_(is_task_running),
  state_(LocalizationTaskState::UNINITIALIZED),
  loc_state_(LocalizationInitializationState::UNKNOWN),
  localization_quality_(false)
{
  init_pose_publisher_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
}

void Localization::execute(
  const std::string & task_id, const geometry_msgs::msg::PoseStamped & init_pose)
{
  autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "RUNNING");
  is_task_running_ = true;

  // Maximum number of initialization retries
  const int MAX_INIT_RETRIES = 5;
  int init_retry_counter = 0;

  while (true) {
    if (cancel_requested_.load()) {
      is_task_running_ = false;
      autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "CANCELLED");
      autoware_bridge_util_->updateTaskStatus(
        task_id, TaskRequestType::REASON, "Cancelled by user");
      RCLCPP_INFO(node_->get_logger(), "Localization task %s cancelled.", task_id.c_str());
      return;
    }

    switch (state_) {
      case LocalizationTaskState::UNINITIALIZED:
        sendCmdGate();
        state_ = LocalizationTaskState::INITIALIZATION;
        RCLCPP_INFO(node_->get_logger(), "Localization state: UNINITIALIZED -> INITIALIZATION");
        break;

      case LocalizationTaskState::INITIALIZATION:
        std::this_thread::sleep_for(500ms);
        // Reset the retry counter when starting initialization
        init_retry_counter = 0;
        loc_state_ = LocalizationInitializationState::INITIALIZED;
        localization_quality_ = true;
        RCLCPP_INFO(node_->get_logger(), "Localization system ready (simulated).");
        pubInitPose(init_pose);
        state_ = LocalizationTaskState::LOCALIZATION;
        RCLCPP_INFO(node_->get_logger(), "Localization state: INITIALIZATION -> LOCALIZATION");
        break;

      case LocalizationTaskState::LOCALIZATION:
        RCLCPP_INFO(
          node_->get_logger(), "Localizing... Quality: %s", localization_quality_ ? "good" : "bad");
        if (loc_state_ == LocalizationInitializationState::INITIALIZED) {
          localization_start_time_ = std::chrono::steady_clock::now();
          state_ = LocalizationTaskState::LOCALIZATION_CHECK;
          RCLCPP_INFO(
            node_->get_logger(), "Localization state: LOCALIZATION -> LOCALIZATION_CHECK");
        } else if (
          loc_state_ == LocalizationInitializationState::UNKNOWN ||
          loc_state_ == LocalizationInitializationState::INITIALIZING) {
          std::this_thread::sleep_for(100ms);  // Prevent busy looping
        } else {
          // Unexpected loc_state_, retry initialization
          init_retry_counter++;
          if (init_retry_counter > MAX_INIT_RETRIES) {
            // Exceeded allowed retries: exit with failure.
            is_task_running_ = false;
            autoware_bridge_util_->updateTaskStatus(
              task_id, TaskRequestType::STATUS, "FAILED", init_retry_counter);
            autoware_bridge_util_->updateTaskStatus(
              task_id, TaskRequestType::REASON, "Exceeded maximum initialization retries");
            RCLCPP_ERROR(
              node_->get_logger(),
              "Localization task %s failed: Exceeded maximum initialization retries",
              task_id.c_str());
            return;
          }
          RCLCPP_WARN(
            node_->get_logger(),
            "Unexpected localization state encountered. Retrying initialization (%d/%d)...",
            init_retry_counter, MAX_INIT_RETRIES);
          state_ = LocalizationTaskState::INITIALIZATION;
        }
        break;

      case LocalizationTaskState::LOCALIZATION_CHECK: {
        try {
          for (int i = 0; i < 20; ++i) {
            if (cancel_requested_.load()) {
              is_task_running_ = false;
              autoware_bridge_util_->updateTaskStatus(
                task_id, TaskRequestType::STATUS, "CANCELLED");
              autoware_bridge_util_->updateTaskStatus(
                task_id, TaskRequestType::REASON, "Cancelled by user");
              RCLCPP_INFO(node_->get_logger(), "Localization task %s cancelled.", task_id.c_str());
              return;
            }
            std::this_thread::sleep_for(100ms);
          }
          if (std::rand() % 5 == 0) {
            throw std::runtime_error("Simulated localization error");
          }
          state_ = LocalizationTaskState::WAIT_UI;
          RCLCPP_INFO(node_->get_logger(), "Localization state: LOCALIZATION_CHECK -> WAIT_UI");
        } catch (const std::exception & e) {
          is_task_running_ = false;
          autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "FAILED");
          autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::REASON, e.what());
          RCLCPP_ERROR(
            node_->get_logger(), "Localization task %s failed: %s", task_id.c_str(), e.what());
          return;
        }
        break;
      }

      case LocalizationTaskState::WAIT_UI:
        autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "SUCCESS");
        RCLCPP_INFO(node_->get_logger(), "Localization completed successfully.");
        is_task_running_ = false;
        return;

      default:
        RCLCPP_WARN(node_->get_logger(), "Unknown localization state encountered.");
        is_task_running_ = false;
        return;
    }

    std::this_thread::sleep_for(100ms);
  }
}
void Localization::request_cancel()
{
  cancel_requested_ = true;
}

bool Localization::isLocalizationQualityAcceptable() const
{
  return localization_quality_;
}

void Localization::sendCmdGate()
{
  RCLCPP_INFO(node_->get_logger(), "Sending command to gate...");
}

void Localization::pubInitPose(const geometry_msgs::msg::PoseStamped & init_pose)
{
  geometry_msgs::msg::PoseWithCovarianceStamped target_pose;
  target_pose.header.frame_id = "map";
  target_pose.header.stamp = node_->now();

  target_pose.pose.pose = init_pose.pose;  // Copy pose from input PoseStamped

  // If needed, add covariance values (defaulting to zero here)
  for (int i = 0; i < 36; i++) {
    target_pose.pose.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;  // Example covariance
  }

  RCLCPP_INFO(
    node_->get_logger(), "Publishing initial pose: (%.2f, %.2f, %.2f)", init_pose.pose.position.x,
    init_pose.pose.position.y, init_pose.pose.position.z);

  init_pose_publisher_->publish(target_pose);
}