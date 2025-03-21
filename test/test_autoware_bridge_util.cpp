#include <gtest/gtest.h>
#include "autoware_bridge/autoware_bridge_util.hpp"
#include "autoware_bridge/srv/get_task_status.hpp"
#include "autoware_bridge/base_task.hpp"
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

// Dummy BaseTask implementation to test active task management.
class DummyTask : public BaseTask {
public:
  void execute(const std::string & task_id, const geometry_msgs::msg::PoseStamped & /*pose*/) override {
    // Dummy implementation.
  }
  void cancelRequested() override {
    // Dummy implementation.
  }
};

class AutowareBridgeUtilTest : public ::testing::Test {
protected:
  void SetUp() override {
    util = std::make_shared<AutowareBridgeUtil>();
  }
  std::shared_ptr<AutowareBridgeUtil> util;
};

TEST_F(AutowareBridgeUtilTest, UpdateTaskStatusTest) {
  std::string task_id = "test_task";
  // First update should create an entry.
  util->updateTaskStatus(task_id, TaskRequestType::STATUS, "PENDING");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "PENDING");

  // Update reason.
  util->updateTaskStatus(task_id, TaskRequestType::REASON, "Starting");
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.reason, "Starting");

  // Update retry number.
  util->updateTaskStatus(task_id, TaskRequestType::RETRIES, "", 3);
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.retry_number, 3);

  // Update total retries.
  util->updateTaskStatus(task_id, TaskRequestType::TOTAL_RETRIES, "", 5);
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.total_retries, 5);

  // Update cancellation status.
  util->updateTaskStatus(task_id, TaskRequestType::CANCEL_STATUS, "CANCELLED");
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.cancel_info.status, "CANCELLED");

  // Update cancellation reason.
  util->updateTaskStatus(task_id, TaskRequestType::CANCEL_REASON, "User requested");
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.cancel_info.reason, "User requested");
}

TEST_F(AutowareBridgeUtilTest, UpdateFailStatusTest) {
  std::string task_id = "fail_task";
  util->updateFailStatus(task_id, "Error occurred");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "FAILED");
  EXPECT_EQ(info.reason, "Error occurred");
}

TEST_F(AutowareBridgeUtilTest, UpdateSuccessStatusTest) {
  std::string task_id = "success_task";
  util->updateSuccessStatus(task_id);
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "SUCCESS");
}

TEST_F(AutowareBridgeUtilTest, UpdateCancellationStatusTest) {
  std::string task_id = "cancel_task";
  util->updateCancellationStatus(task_id, "Cancelled by user");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "CANCELLED");
  EXPECT_EQ(info.reason, "Cancelled by user");
}

TEST_F(AutowareBridgeUtilTest, UpdateRunningStatusWithRetriesTest) {
  std::string task_id = "running_task";
  int total_retries = 4;
  util->updateRunningStatusWithRetries(task_id, total_retries);
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "RUNNING");
  EXPECT_EQ(info.total_retries, total_retries);
}

TEST_F(AutowareBridgeUtilTest, UpdateHaltStatusTest) {
  std::string task_id = "halt_task";
  util->updateHaltStatus(task_id, "Halted due to timeout");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "HALTED");
  EXPECT_EQ(info.reason, "Halted due to timeout");
}

TEST_F(AutowareBridgeUtilTest, IsTaskActiveTest) {
  std::string task_id = "active_task";
  // Initially, no task exists.
  EXPECT_FALSE(util->isTaskActive(task_id));
  
  util->updateTaskStatus(task_id, TaskRequestType::STATUS, "PENDING");
  EXPECT_TRUE(util->isTaskActive(task_id));
}

TEST_F(AutowareBridgeUtilTest, GetActiveTaskIdTest) {
  // When no tasks exist, getActiveTaskId() should return "NO_ACTIVE_TASK".
  EXPECT_EQ(util->getActiveTaskId(), "NO_ACTIVE_TASK");
  
  // Add a task.
  std::string task_id = "active_task";
  util->updateTaskStatus(task_id, TaskRequestType::STATUS, "PENDING");
  EXPECT_EQ(util->getActiveTaskId(), task_id);
}

TEST_F(AutowareBridgeUtilTest, IsActiveTaskIdEmptyTest) {
  EXPECT_TRUE(util->isActiveTaskIdEmpty());
  std::string task_id = "some_task";
  util->updateTaskStatus(task_id, TaskRequestType::STATUS, "PENDING");
  EXPECT_FALSE(util->isActiveTaskIdEmpty());
}

TEST_F(AutowareBridgeUtilTest, GetTaskStatusTest) {
  std::string task_id = "status_task";
  // For a non-existent task, getTaskStatus() returns default TaskInfo with empty status.
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "");
  
  util->updateTaskStatus(task_id, TaskRequestType::STATUS, "PENDING");
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "PENDING");
}

TEST_F(AutowareBridgeUtilTest, ActiveTaskPointerTest) {
  auto dummy = std::make_shared<DummyTask>();
  util->setActiveTask(dummy);
  EXPECT_EQ(util->getActiveTaskPointer(), dummy);
  util->clearActiveTask();
  EXPECT_EQ(util->getActiveTaskPointer(), nullptr);
}

TEST_F(AutowareBridgeUtilTest, HandleStatusRequestTest) {
  auto request = std::make_shared<autoware_bridge::srv::GetTaskStatus::Request>();
  auto response = std::make_shared<autoware_bridge::srv::GetTaskStatus::Response>();
  request->task_id = "service_task";
  
  // With no active task entry, handleStatusRequest should set response to REJECTED.
  util->handleStatusRequest(request, response);
  EXPECT_EQ(response->status, "REJECTED");
  EXPECT_EQ(response->reason, "Requested task_id is not the last active one");
  EXPECT_EQ(response->retry_number, 0);
  EXPECT_EQ(response->total_retries, 0);

  // Update the task status and then handle the request.
  util->updateTaskStatus(request->task_id, TaskRequestType::STATUS, "RUNNING");
  util->updateTaskStatus(request->task_id, TaskRequestType::RETRIES, "", 2);
  util->updateTaskStatus(request->task_id, TaskRequestType::TOTAL_RETRIES, "", 5);

  util->handleStatusRequest(request, response);
  EXPECT_EQ(response->status, "RUNNING");
  EXPECT_EQ(response->retry_number, 2);
  EXPECT_EQ(response->total_retries, 5);
}

// New Test: Concurrent Access / Thread Safety
TEST_F(AutowareBridgeUtilTest, ConcurrentUpdateTest) {
  std::string task_id = "concurrent_task";
  const int num_threads = 10;
  const int num_iterations = 100;
  std::vector<std::thread> threads;

  auto updateFunc = [this, &task_id, num_iterations]() {
    for (int i = 0; i < num_iterations; ++i) {
      // Alternate between updating status and retries.
      util->updateTaskStatus(task_id, TaskRequestType::STATUS, "RUNNING");
      util->updateTaskStatus(task_id, TaskRequestType::RETRIES, "", i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  };

  // Launch multiple threads updating the same task concurrently.
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back(updateFunc);
  }

  for (auto & t : threads) {
    t.join();
  }

  // Final value might be non-deterministic for retries; verify status is updated.
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "RUNNING");
  // The retry number should be within the expected range.
  EXPECT_GE(info.retry_number, 0);
  EXPECT_LE(info.retry_number, num_iterations - 1);
}

// New Test: Multiple Tasks Scenario
TEST_F(AutowareBridgeUtilTest, MultipleTasksTest) {
  // Insert multiple tasks.
  std::vector<std::string> task_ids = {"task1", "task2", "task3"};
  for (const auto & id : task_ids) {
    util->updateTaskStatus(id, TaskRequestType::STATUS, "PENDING");
  }
  
  // Check each task is active.
  for (const auto & id : task_ids) {
    EXPECT_TRUE(util->isTaskActive(id));
  }
  
  // getActiveTaskId() returns the first found task; ensure it is one of the tasks.
  std::string activeTaskId = util->getActiveTaskId();
  std::cout << "Checking task2 status: " << util->isTaskActive("task2") << std::endl;
  std::cout << "Checking task3 status: " << util->isTaskActive("task3") << std::endl;
  std::cout << "Currently active task ID: " << util->getActiveTaskId() << std::endl;
  EXPECT_TRUE(std::find(task_ids.begin(), task_ids.end(), activeTaskId) != task_ids.end());
}

// New Test: Invalid Request Type (simulate out-of-range enum value)
TEST_F(AutowareBridgeUtilTest, InvalidRequestTypeTest) {
  std::string task_id = "invalid_task";
  // Cast an out-of-range integer to TaskRequestType.
  TaskRequestType invalidType = static_cast<TaskRequestType>(999);
  // Call updateTaskStatus with an invalid request type.
  // It should trigger the default case and log a warning.
  // There is no change to TaskInfo for an invalid request.
  util->updateTaskStatus(task_id, invalidType, "IGNORED");
  TaskInfo info = util->getTaskStatus(task_id);
  // Since no valid update occurred, status should remain default.
  EXPECT_EQ(info.status, "");
}
