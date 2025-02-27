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
  autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::TOTAL_RETRIES, "", 5);
  is_task_running_ = true;

  // Maximum number of initialization retries
  const int MAX_INIT_RETRIES = 5;
  int retry_counter = 0;

  while (true) {
    mutex.lock();

    if (cancel_requested_.load()) {
      autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "CANCELLED");
      autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::REASON, "Cancelled by user");
      RCLCPP_INFO(node_->get_logger(), "Localization task %s cancelled.", task_id.c_str());
      is_task_running_ = false;
      return;
    }

    if (retry_counter >= MAX_INIT_RETRIES){
      //FAILURE

      updateFailStatus(task_id, "Max retries elapsed");
      // autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "FAILED");
      // autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::REASON, "Max retries elapsed");
      is_task_running_ = false;
      break;
    }

    if (success){
      //SUCCESS
      autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "SUCCESS");
      is_task_running_ = false;
      break;
    }

    switch (state_) {
      // sendCmdGate(); // check this whether required or not

      case LocalizationTaskState::INITIALIZATION:
        if (retry_counter == 1) {
          autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "RETRYING");
        }
        retry_counter++;
        autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::RETRIES, "", retry_counter);

        pubInitPose(init_pose);
        std::this_thread::sleep_for(500ms);
        state_ = LocalizationTaskState::LOCALIZATION;

      case LocalizationTaskState::LOCALIZATION:

        switch (this->localization_state_) {
          case LocalizationInitializationState::UNINITIALIZED:
            state_ = LocalizationTaskState::INITIALIZATION;
            break;

          case LocalizationInitializationState::INITIALIZED:
            localization_start_time_ = this->get_clock()->now();
            state_ = LocalizationTaskState::LOCALIZATION_CHECK;
            break;
          default:
            RCLCPP_INFO(node_->get_logger(), "Localization state: %s", localization_state_);
        }

      case LocalizationTaskState::LOCALIZATION_CHECK:
        
        if ( this->get_clock()->now().seconds() - localization_start_time_.seconds() > LOC_WAIT_TIMEOUT_S ) {
          if (this->localization_quality_) {
            bool success = true;
            // autoware_bridge_util_->updateTaskStatus(task_id, TaskRequestType::STATUS, "SUCCESS");
            // break;
          } 
          else {
            RCLCPP_INFO(this->get_logger(), "Localisation quality is poor, retrying localization");
            this->ftd_state_ = LocalizationInitializationState::INITIALIZATION;
          }
        } 
        else {
          RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000, "Waiting for localization result");
        }

      default:
        break;
    }
  }
  // std::this_thread::sleep_for(100ms);
}
void Localization::request_cancel()
{
  mutex.lock();
  cancel_requested_ = true;
}

// make it a callback to update the status
bool Localization::isLocalizationQualityAcceptable() const
{
  return localization_quality_;
}

// check it's usability
// void Localization::sendCmdGate()
// {
//   RCLCPP_INFO(node_->get_logger(), "Sending command to gate...");
// }

void Localization::pubInitPose(const geometry_msgs::msg::PoseStamped & init_pose)
{
  // geometry_msgs::msg::PoseStamped target_pose;
  // target_pose = init_pose;  // Copy pose from input PoseStamped
  init_pose_publisher_->publish(init_pose);
}