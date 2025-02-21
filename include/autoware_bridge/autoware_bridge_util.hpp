#ifndef AUTOWARE_BRIDGE_UTIL_HPP
#define AUTOWARE_BRIDGE_UTIL_HPP

#include "base_task.hpp"
#include "rclcpp/rclcpp.hpp"

#include <map>
#include <memory>  // For std::shared_ptr
#include <mutex>
#include <string>
// Include the service headers for GetTaskStatus and CancelTask
// #include "autoware_bridge/srv/cancel_task.hpp"
#include "autoware_bridge/srv/get_task_status.hpp"

#define EMPTY_STRING ""

enum class TaskRequestType { STATUS, REASON, RETRIES, TOTAL_RETRIES, CANCEL_STATUS, CANCEL_REASON };

struct TaskCancellationInfo
{
  std::string status = EMPTY_STRING;
  std::string reason = EMPTY_STRING;
};

struct TaskInfo
{
  std::string status =
    EMPTY_STRING;  // rejected -> pending -> running -> retrying -> success | failed -> cancelled
  std::string reason = EMPTY_STRING;  // Failure reason or rejection explanation
  int32_t retry_number = 0;
  int32_t total_retries = 0;
  TaskCancellationInfo cancel_info;
};

class AutowareBridgeUtil
{
public:
  AutowareBridgeUtil() : active_task_(nullptr) {}

  void updateTaskStatus(
    const std::string & task_id, TaskRequestType request_type, const std::string & value,
    int number = 0);

  bool isTaskActive(const std::string & task_id);
  std::string getActiveTaskId();
  bool isActiveTaskIdEmpty();
  TaskInfo getTaskStatus(const std::string & task_id);

  void setActiveTask(std::shared_ptr<BaseTask> task_ptr);
  void clearActiveTask();
  std::shared_ptr<BaseTask> getActiveTaskPointer();

  void handleStatusRequest(
    const std::shared_ptr<autoware_bridge::srv::GetTaskStatus_Request> request,
    std::shared_ptr<autoware_bridge::srv::GetTaskStatus_Response> response);

private:
  std::mutex task_mutex_;
  std::map<std::string, TaskInfo> task_map_;
  std::shared_ptr<BaseTask> active_task_;
};

#endif  // AUTOWARE_BRIDGE_UTIL_HPP
