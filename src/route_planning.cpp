#include "autoware_bridge/route_planning.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

RoutePlanning::RoutePlanning(
  rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util,
  std::atomic<bool> & is_task_running)
: node_(node),
  autoware_bridge_util_(autoware_bridge_util),
  cancel_requested_(false),
  is_task_running_(is_task_running)
{
}

void RoutePlanning::execute(const std::string & task_id, const geometry_msgs::msg::PoseStamped & pose)
{
  autoware_bridge_util_->update_task_status(task_id, "RUNNING");

  /* while (processing) {  // Example processing loop
    if (cancel_requested_) {  // Check if cancellation was requested
      RCLCPP_INFO(rclcpp::get_logger("RoutePlanning"), "SetGoal task cancelled.");
      return;  // Exit early
    }

    // Localization logic here...
  } */

  try {
    std::this_thread::sleep_for(std::chrono::seconds(3));  // Simulated processing
    if (rand() % 4 == 0) {
      throw std::runtime_error("Simulated set goal error");
    }

    autoware_bridge_util_->update_task_status(task_id, "SUCCESS");
  } catch (const std::exception & e) {
    autoware_bridge_util_->update_task_status(task_id, "ERROR");
    RCLCPP_ERROR(node_->get_logger(), "Set goal task failed: %s", e.what());
  }
}

void RoutePlanning::request_cancel()
{
  // is_canceled_ = true; // we can use any flag like this in above function to not execute or
  // we can write something here to send stop request.
}