#ifndef AUTOWARE_BRIDGE_UTIL_HPP
#define AUTOWARE_BRIDGE_UTIL_HPP

#include "rclcpp/rclcpp.hpp"

#include <map>
#include <memory>  // For std::shared_ptr
#include <mutex>
#include <string>

// Include the service headers for GetTaskStatus and CancelTask
#include "autoware_bridge/srv/cancel_task.hpp"
#include "autoware_bridge/srv/get_task_status.hpp"

class AutowareBridgeUtil
{
public:
  AutowareBridgeUtil() = default;
  std::string generate_task_id(const std::string & task_name);
  void update_task_status(const std::string & task_id, const std::string & status);
  std::string get_task_status(const std::string & task_id);
  std::string get_active_task();
  bool cancel_task(const std::string & task_id);

  void handle_status_request(
    const std::shared_ptr<autoware_bridge::srv::GetTaskStatus_Request> request,
    std::shared_ptr<autoware_bridge::srv::GetTaskStatus_Response> response);
  void handle_cancel_request(
    const std::shared_ptr<autoware_bridge::srv::CancelTask_Request> request,
    std::shared_ptr<autoware_bridge::srv::CancelTask_Response> response);

private:
  std::mutex task_mutex_;
  std::map<std::string, std::string> task_status_;
};

#endif  // AUTOWARE_BRIDGE_UTIL_HPP
