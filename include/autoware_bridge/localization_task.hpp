#ifndef LOCALIZATION_TASK_HPP
#define LOCALIZATION_TASK_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"

#include <rclcpp/rclcpp.hpp>

class LocalizationTask
{
public:
  LocalizationTask(rclcpp::Node::SharedPtr node, AutowareBridgeUtil & autoware_bridge_util);
  void execute(const std::string & task_id);  // This is just execution, NOT task management
  void cancel();                              // New method to cancel the task

private:
  rclcpp::Node::SharedPtr node_;
  AutowareBridgeUtil & autoware_bridge_util_;
};

#endif  // LOCALIZATION_TASK_HPP
