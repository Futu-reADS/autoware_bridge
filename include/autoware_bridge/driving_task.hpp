#ifndef DRIVING_TASK_HPP
#define DRIVING_TASK_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"
#include "base_task.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>

class DrivingTask : public BaseTask
{
public:
  DrivingTask(
    rclcpp::Node::SharedPtr node, std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util,
    std::atomic<bool> & is_task_running);
  void execute(const std::string & task_id, const geometry_msgs::msg::PoseStamped & pose)
    override;                      // Executes Driving
  void request_cancel() override;  // Requests task cancellation

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<AutowareBridgeUtil> autoware_bridge_util_;  // Use shared_ptr instead of reference
  std::atomic<bool> cancel_requested_;
  std::atomic<bool> & is_task_running_;
};

#endif  // LOCALIZATION_TASK_HPP
