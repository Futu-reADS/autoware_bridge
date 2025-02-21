#include "autoware_bridge/localization.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

Localization::Localization(
  rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util,
  std::atomic<bool> & is_task_running)
: node_(node),
  autoware_bridge_util_(std::move(autoware_bridge_util)),  // Move shared_ptr for efficiency
  cancel_requested_(false),
  is_task_running_(is_task_running)
{
}

void Localization::execute(
  const std::string & task_id, const geometry_msgs::msg::PoseStamped & pose)
{
  autoware_bridge_util_->update_task_status(task_id, "RUNNING");

  /* while (processing) {  // Example processing loop
    if (cancel_requested_) {  // Check if cancellation was requested
      RCLCPP_INFO(rclcpp::get_logger("Localization"), "Localization task cancelled.");
      return;  // Exit early
    }

    // Localization logic here...
  } */

  try {
    for (int i = 0; i < 20; ++i) {  // Simulating localization steps
      if (cancel_requested_) {
        is_task_running_ = false;
        autoware_bridge_util_->update_task_status(task_id, "CANCELLED", "Cancelled by user");
        RCLCPP_INFO(node_->get_logger(), "Localization task %s cancelled.", task_id.c_str());
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (rand() % 5 == 0) throw std::runtime_error("Simulated localization error");

    autoware_bridge_util_->update_task_status(task_id, "SUCCESS");
    is_task_running_ = false;
  } catch (const std::exception & e) {
    autoware_bridge_util_->update_task_status(task_id, "FAILED", e.what());
    RCLCPP_ERROR(node_->get_logger(), "Localization task %s failed: %s", task_id.c_str(), e.what());
    is_task_running_ = false;
  }
}

void Localization::request_cancel()
{
  cancel_requested_ = true;
}
