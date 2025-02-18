#ifndef DRIVING_TASK_HPP
#define DRIVING_TASK_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"
#include "base_task.hpp"

#include <rclcpp/rclcpp.hpp>

class DrivingTask : public BaseTask
{
public:
  DrivingTask(
    rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util,
    std::atomic<bool> & is_task_running);
  void execute(const std::string & task_id);  // This is just execution, NOT task management
  void request_cancel();                      // New method to cancel the task

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util_;  // Use shared_ptr instead of reference
  std::atomic<bool> cancel_requested_;
  std::atomic<bool> & is_task_running_;
};

#endif  // LOCALIZATION_TASK_HPP
