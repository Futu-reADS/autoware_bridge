#include "autoware_bridge/set_goal_task.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

SetGoalTask::SetGoalTask(rclcpp::Node::SharedPtr node, AutowareBridgeUtil & autoware_bridge_util)
: node_(node), autoware_bridge_util_(autoware_bridge_util)
{
}

void SetGoalTask::execute(const std::string & task_id)
{
  autoware_bridge_util_.update_task_status(task_id, "RUNNING");

  try {
    std::this_thread::sleep_for(std::chrono::seconds(3));  // Simulated processing
    if (rand() % 4 == 0) {
      throw std::runtime_error("Simulated set goal error");
    }

    autoware_bridge_util_.update_task_status(task_id, "SUCCESS");
  } catch (const std::exception & e) {
    autoware_bridge_util_.update_task_status(task_id, "ERROR");
    RCLCPP_ERROR(node_->get_logger(), "Set goal task failed: %s", e.what());
  }
}

void SetGoalTask::cancel()
{
  // is_canceled_ = true; // we can use any flag like this in above function to not execute or
  // we can write something here to send stop request.
}