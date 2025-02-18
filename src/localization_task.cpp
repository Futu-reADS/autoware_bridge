#include "autoware_bridge/localization_task.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

LocalizationTask::LocalizationTask(
  rclcpp::Node::SharedPtr node, AutowareBridgeUtil & autoware_bridge_util,
  std::atomic<bool> & is_task_running)
: node_(node),
  autoware_bridge_util_(autoware_bridge_util),
  cancel_requested_(false),
  is_task_running_(is_task_running)
{
}

void LocalizationTask::execute(const std::string & task_id)
{
  autoware_bridge_util_.update_task_status(task_id, "RUNNING");

  /* while (processing) {  // Example processing loop
    if (cancel_requested_) {  // Check if cancellation was requested
      RCLCPP_INFO(rclcpp::get_logger("LocalizationTask"), "Localization task cancelled.");
      return;  // Exit early
    }

    // Localization logic here...
  } */
  try {
    for (int i = 0; i < 20; ++i) {  // Simulating localization steps
      if (cancel_requested_) {      // Check if cancellation was requested
        is_task_running_ = false;
        autoware_bridge_util_.update_task_status(task_id, "CANCELLED");
        RCLCPP_INFO(node_->get_logger(), "Localization task cancelled.");
        return;  // Exit early
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Simulate processing
    }

    if (rand() % 5 == 0) throw std::runtime_error("Simulated localization error");

    autoware_bridge_util_.update_task_status(task_id, "SUCCESS");
    is_task_running_ = false;  // Mark task as completed
  } catch (const std::exception & e) {
    autoware_bridge_util_.update_task_status(task_id, "ERROR");
    RCLCPP_ERROR(node_->get_logger(), "Localization task failed: %s", e.what());
    is_task_running_ = false;  // Ensure the task is marked as not running after error
  }
}

void LocalizationTask::request_cancel()
{
  cancel_requested_ = true;
}