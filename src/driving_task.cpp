#include "autoware_bridge/driving_task.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

DrivingTask::DrivingTask(rclcpp::Node::SharedPtr node, AutowareBridgeUtil & autoware_bridge_util)
: node_(node), autoware_bridge_util_(autoware_bridge_util)
{
}

void DrivingTask::execute(const std::string & task_id)
{
  autoware_bridge_util_.update_task_status(task_id, "RUNNING");
  // write your localization logic here and set the status and response.
  try {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    if (rand() % 5 == 0) throw std::runtime_error("Simulated localization error");

    autoware_bridge_util_.update_task_status(task_id, "SUCCESS");
  } catch (const std::exception & e) {
    autoware_bridge_util_.update_task_status(task_id, "ERROR");
    RCLCPP_ERROR(node_->get_logger(), "Localization task failed: %s", e.what());
  }
}

void DrivingTask::request_cancel()
{
  // is_canceled_ = true; // we can use any flag like this in above function to not execute or
  // we can write something here to send stop request.
}