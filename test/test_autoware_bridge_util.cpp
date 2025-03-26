// test_autoware_bridge_util.cpp

#include "autoware_bridge/autoware_bridge_util.hpp"
#include "autoware_bridge/base_task.hpp"
#include "autoware_bridge/srv/get_task_status.hpp"

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// Dummy BaseTask implementation to test active task management.
class DummyTask : public BaseTask {
public:
  void execute(const std::string & task_id, const geometry_msgs::msg::PoseStamped & /*pose*/) override {
    (void)task_id; // Dummy implementation.
  }
  void cancel() override {
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

// Test: Update Task Status and Field Updates.
TEST_F(AutowareBridgeUtilTest, UpdateTaskStatusTest) {
  std::string task_id = "test_task";
  
  // Create the task entry.
  util->updateTaskId(task_id);
  
  // Update main status and reason.
  util->updateTaskStatus(task_id, "PENDING", "Starting process");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "PENDING");
  EXPECT_EQ(info.reason, "Starting process");

  // Update retry number.
  util->updateTaskRetries(task_id, 3);
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.retry_number, 3);

  // Update total retries.
  util->updateTask(task_id, TaskRequestType::TOTAL_RETRIES, "", 5);
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.total_retries, 5);

  // Update cancellation:
  // For status "CANCELLED", updateTaskStatus should update both main and cancellation info.
  util->updateTaskStatus(task_id, "CANCELLED", "User cancelled");
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "CANCELLED");
  EXPECT_EQ(info.reason, "User cancelled");
  EXPECT_EQ(info.cancel_info.status, "CANCELLED");
  EXPECT_EQ(info.cancel_info.reason, "User cancelled");
}

// Test: Update Fail Status via updateTaskStatus with "FAILED".
TEST_F(AutowareBridgeUtilTest, UpdateFailStatusTest) {
  std::string task_id = "fail_task";
  util->updateTaskId(task_id);
  util->updateTaskStatus(task_id, "FAILED", "Error occurred");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "FAILED");
  EXPECT_EQ(info.reason, "Error occurred");
  // For FAILED, cancellation auto-updates: status "REJECTED", reason "FAILED"
  EXPECT_EQ(info.cancel_info.status, "REJECTED");
  EXPECT_EQ(info.cancel_info.reason, "FAILED");
}

// Test: Update Success Status via updateTaskStatus with "SUCCESS".
TEST_F(AutowareBridgeUtilTest, UpdateSuccessStatusTest) {
  std::string task_id = "success_task";
  util->updateTaskId(task_id);
  util->updateTaskStatus(task_id, "SUCCESS", "");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "SUCCESS");
  // For SUCCESS, cancellation auto-updates: status "REJECTED", reason "SUCCESS"
  EXPECT_EQ(info.cancel_info.status, "REJECTED");
  EXPECT_EQ(info.cancel_info.reason, "SUCCESS");
}

// Test: Update Halt Status.
TEST_F(AutowareBridgeUtilTest, UpdateHaltStatusTest) {
  std::string task_id = "halt_task";
  util->updateTaskId(task_id);
  util->updateTaskStatus(task_id, "HALTED", "Halted due to timeout");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "HALTED");
  EXPECT_EQ(info.reason, "Halted due to timeout");
}

// Test: Check if Task is Active.
TEST_F(AutowareBridgeUtilTest, IsTaskActiveTest) {
  std::string task_id = "active_task";
  EXPECT_FALSE(util->isTaskActive(task_id));
  
  util->updateTaskId(task_id);
  util->updateTaskStatus(task_id, "PENDING", "Waiting");
  EXPECT_TRUE(util->isTaskActive(task_id));
}

// Test: Retrieve Active Task ID.
TEST_F(AutowareBridgeUtilTest, GetActiveTaskIdTest) {
  EXPECT_EQ(util->getActiveTaskId(), "NO_ACTIVE_TASK");
  
  std::string task_id = "active_task";
  util->updateTaskId(task_id);
  EXPECT_EQ(util->getActiveTaskId(), task_id);
}

// Test: Retrieve Task Status.
TEST_F(AutowareBridgeUtilTest, GetTaskStatusTest) {
  std::string task_id = "status_task";
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "");
  
  util->updateTaskId(task_id);
  util->updateTaskStatus(task_id, "PENDING", "Waiting for execution");
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "PENDING");
  EXPECT_EQ(info.reason, "Waiting for execution");
}

// Test: Active Task Pointer Management.
TEST_F(AutowareBridgeUtilTest, ActiveTaskPointerTest) {
  auto dummy = std::make_shared<DummyTask>();
  util->setActiveTaskPtr(dummy);
  EXPECT_EQ(util->getActiveTaskPtr(), dummy);
  util->clearActiveTaskPtr();
  EXPECT_EQ(util->getActiveTaskPtr(), nullptr);
}

// Test: Handle Status Request Service.
TEST_F(AutowareBridgeUtilTest, HandleStatusRequestTest) {
  auto request = std::make_shared<autoware_bridge::srv::GetTaskStatus::Request>();
  auto response = std::make_shared<autoware_bridge::srv::GetTaskStatus::Response>();
  request->task_id = "service_task";

  // With no active task, expect rejection.
  util->handleStatusRequestSrvc(request, response);
  EXPECT_EQ(response->status, "REJECTED");
  EXPECT_EQ(response->reason, "Requested task_id is not the last active one");
  EXPECT_EQ(response->retry_number, 0);
  EXPECT_EQ(response->total_retries, 0);

  // Create and update a task.
  util->updateTaskId(request->task_id);
  util->updateTaskStatus(request->task_id, "RUNNING", "");
  util->updateTaskRetries(request->task_id, 2);
  util->updateTask(request->task_id, TaskRequestType::TOTAL_RETRIES, "", 5);

  util->handleStatusRequestSrvc(request, response);
  EXPECT_EQ(response->status, "RUNNING");
  EXPECT_EQ(response->retry_number, 2);
  EXPECT_EQ(response->total_retries, 5);
  // Reason is empty as updated.
}

// Test: Concurrent Update / Thread Safety.
TEST_F(AutowareBridgeUtilTest, ConcurrentUpdateTest) {
  std::string task_id = "concurrent_task";
  const int num_threads = 10;
  const int num_iterations = 100;
  std::vector<std::thread> threads;

  util->updateTaskId(task_id);

  auto updateFunc = [this, &task_id, num_iterations]() {
    for (int i = 0; i < num_iterations; ++i) {
      util->updateTaskStatus(task_id, "RUNNING", "");
      util->updateTaskRetries(task_id, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  };

  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back(updateFunc);
  }
  for (auto & t : threads) {
    t.join();
  }

  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "RUNNING");
  EXPECT_GE(info.retry_number, 0);
  EXPECT_LE(info.retry_number, num_iterations - 1);
}

// Test: Multiple Tasks Scenario (only one active task allowed).
TEST_F(AutowareBridgeUtilTest, MultipleTasksTest) {
  // Add first task.
  util->updateTaskId("task1");
  EXPECT_TRUE(util->isTaskActive("task1"));

  // Adding a second task clears previous tasks.
  util->updateTaskId("task2");
  EXPECT_FALSE(util->isTaskActive("task1"));
  EXPECT_TRUE(util->isTaskActive("task2"));
}

// Test: Get Task Status for a non-existent task.
TEST_F(AutowareBridgeUtilTest, GetTaskStatusNonExistentTest) {
  std::string task_id = "non_existent_task";
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "");
}
