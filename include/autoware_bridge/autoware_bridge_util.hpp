#ifndef AUTOWARE_BRIDGE_UTIL_HPP
#define AUTOWARE_BRIDGE_UTIL_HPP

#include "autoware_bridge/srv/get_task_status.hpp"
#include "autoware_bridge/base_task.hpp"
#include "rclcpp/rclcpp.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>


#define EMPTY_STRING ""

using autoware_bridge::BaseTask;

enum class TaskRequestType { STATUS, REASON, RETRIES, TOTAL_RETRIES, CANCEL_STATUS, CANCEL_REASON };

struct TaskInfo
{
  TaskInfo(int max_retries)
  {
    status = "PENDING";
    retry_number = 0;
    total_retries = max_retries;
  }

  std::string status;
  int32_t retry_number;
  int32_t total_retries;
  std::string cancel_status;
};

class AutowareBridgeUtil
{
public:
  AutowareBridgeUtil() : active_task_(nullptr) {}
  void updateTask(
    const std::string & task_id, TaskRequestType request_type, const std::string & value,
    int number = 0);
  void updateTaskId(const std::string & task_id);

  void updateTaskStatus(const std::string & task_id, const std::string & status);
  void updateTaskRetries(const std::string & task_id, int retryNumber);

  bool isTaskActive(const std::string & task_id);
  std::string getActiveTaskId();
  TaskInfo getTaskStatus(const std::string & task_id);

  void setActiveTaskPtr(std::shared_ptr<BaseTask> task_ptr);
  void clearActiveTaskPtr();
  std::shared_ptr<BaseTask> getActiveTaskPtr();

  void handleStatusRequestSrvc(
    const std::shared_ptr<autoware_bridge::srv::GetTaskStatus_Request> request,
    std::shared_ptr<autoware_bridge::srv::GetTaskStatus_Response> response);

private:
  std::mutex task_mutex_;
  std::map<std::string, TaskInfo> task_map_;
  std::shared_ptr<BaseTask> active_task_;
};

#endif  // AUTOWARE_BRIDGE_UTIL_HPP
