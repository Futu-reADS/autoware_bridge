#ifndef SET_GOAL_TASK_HPP
#define SET_GOAL_TASK_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"

#include <rclcpp/rclcpp.hpp>

class SetGoalTask
{
public:
  SetGoalTask(
    rclcpp::Node::SharedPtr node, AutowareBridgeUtil & autoware_bridge_util,
    std::atomic<bool> & is_task_running);
  void execute(const std::string & task_id);  // Just executes task, NO thread management
  void request_cancel();

private:
  rclcpp::Node::SharedPtr node_;
  AutowareBridgeUtil & autoware_bridge_util_;
  std::atomic<bool> cancel_requested_;
  std::atomic<bool> & is_task_running_;
};

#endif  // SET_GOAL_TASK_HPP
