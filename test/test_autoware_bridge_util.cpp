#include "autoware_bridge/autoware_bridge_util.hpp"
#include "autoware_bridge/base_task.hpp"
#include "autoware_bridge/srv/get_task_status.hpp"

#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

// DummyTask is a dummy implementation of BaseTask used for testing active task pointer management.
class DummyTask : public BaseTask {
public:
  void execute(const std::string & task_id, const geometry_msgs::msg::PoseStamped & pose) override {
    (void)task_id;
    (void)pose;
  }
  void cancel() override {}
};

// Test fixture for AutowareBridgeUtil.
class AutowareBridgeUtilTest : public ::testing::Test {
protected:
  void SetUp() override {
    util = std::make_shared<AutowareBridgeUtil>();
  }
  std::shared_ptr<AutowareBridgeUtil> util;
};

//-------------------------------------------------------------
// Basic Functionality Tests
//-------------------------------------------------------------

// Test: UpdateTaskId and verifying getActiveTaskId returns the new task ID.
TEST_F(AutowareBridgeUtilTest, UpdateTaskIdAndActiveTaskIdTest) {
  // Initially, there is no active task.
  EXPECT_EQ(util->getActiveTaskId(), "NO_ACTIVE_TASK");

  std::string task_id = "task_1";
  util->updateTaskId(task_id);
  EXPECT_EQ(util->getActiveTaskId(), task_id);
}

// Test: UpdateTaskStatus with a normal status ("RUNNING").
// This verifies that only the task's status is updated without affecting cancel status.
TEST_F(AutowareBridgeUtilTest, UpdateTaskStatusNormalTest) {
  std::string task_id = "task_1";
  util->updateTaskId(task_id);

  util->updateTaskStatus(task_id, "RUNNING");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "RUNNING");
  EXPECT_EQ(info.cancel_status, "");
}

// Test: UpdateTaskStatus with "FAILED".
// This should update the status and auto-update cancel_status to "FAILED_DUE_TO_TASK_FAILURE".
TEST_F(AutowareBridgeUtilTest, UpdateTaskStatusFailedTest) {
  std::string task_id = "task_1";
  util->updateTaskId(task_id);

  util->updateTaskStatus(task_id, "FAILED");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "FAILED");
  EXPECT_EQ(info.cancel_status, "FAILED_DUE_TO_TASK_FAILURE");
}

// Test: UpdateTaskStatus with "TIMEOUT".
// This should update the status and auto-update cancel_status similarly to "FAILED".
TEST_F(AutowareBridgeUtilTest, UpdateTaskStatusTimeoutTest) {
  std::string task_id = "task_1";
  util->updateTaskId(task_id);

  util->updateTaskStatus(task_id, "TIMEOUT");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "TIMEOUT");
  EXPECT_EQ(info.cancel_status, "FAILED_DUE_TO_TASK_FAILURE");
}

// Test: UpdateTaskStatus with "SUCCESS".
// This should update the status and auto-update cancel_status to "FAILED_DUE_TO_TASK_SUCCESS".
TEST_F(AutowareBridgeUtilTest, UpdateTaskStatusSuccessTest) {
  std::string task_id = "task_1";
  util->updateTaskId(task_id);

  util->updateTaskStatus(task_id, "SUCCESS");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "SUCCESS");
  EXPECT_EQ(info.cancel_status, "FAILED_DUE_TO_TASK_SUCCESS");
}

// Test: UpdateTaskStatus with "CANCELLED".
// This should update both status and cancel_status to "CANCELLED".
TEST_F(AutowareBridgeUtilTest, UpdateTaskStatusCancelledTest) {
  std::string task_id = "task_1";
  util->updateTaskId(task_id);

  util->updateTaskStatus(task_id, "CANCELLED");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "CANCELLED");
  EXPECT_EQ(info.cancel_status, "CANCELLED");
}

// Test: UpdateTaskRetries updates the retry count correctly.
TEST_F(AutowareBridgeUtilTest, UpdateTaskRetriesTest) {
  std::string task_id = "task_1";
  util->updateTaskId(task_id);

  util->updateTaskRetries(task_id, 5);
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.retry_number, 5);
}

// Test: GetTaskStatus for a non-existent task should return a default TaskInfo.
// Note: The default TaskInfo is constructed with max_retries = 3.
TEST_F(AutowareBridgeUtilTest, GetTaskStatusDefaultTest) {
  std::string task_id = "non_existent";
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "PENDING");
  EXPECT_EQ(info.retry_number, 0);
  EXPECT_EQ(info.total_retries, 3);
}

// Test: isTaskActive returns false for non-existent tasks and true for existing tasks.
TEST_F(AutowareBridgeUtilTest, IsTaskActiveTest) {
  std::string task_id = "task_1";
  EXPECT_FALSE(util->isTaskActive(task_id));

  util->updateTaskId(task_id);
  EXPECT_TRUE(util->isTaskActive(task_id));
}

// Test: Service request handling when the task is not active.
// It should return a response with status "REJECTED" and zero retry values.
TEST_F(AutowareBridgeUtilTest, HandleStatusRequestServiceTest_NonActive) {
  auto request = std::make_shared<autoware_bridge::srv::GetTaskStatus::Request>();
  auto response = std::make_shared<autoware_bridge::srv::GetTaskStatus::Response>();
  request->task_id = "non_existent";

  util->handleStatusRequestSrvc(request, response);
  EXPECT_EQ(response->status, "REJECTED");
  EXPECT_EQ(response->retry_number, 0);
  EXPECT_EQ(response->total_retries, 0);
}

// Test: Service request handling when the task is active.
// It should return the task's current status and retry information.
TEST_F(AutowareBridgeUtilTest, HandleStatusRequestServiceTest_Active) {
  auto request = std::make_shared<autoware_bridge::srv::GetTaskStatus::Request>();
  auto response = std::make_shared<autoware_bridge::srv::GetTaskStatus::Response>();
  std::string task_id = "task_active";
  request->task_id = task_id;

  util->updateTaskId(task_id);
  util->updateTaskStatus(task_id, "RUNNING");
  util->updateTaskRetries(task_id, 2);

  util->handleStatusRequestSrvc(request, response);
  EXPECT_EQ(response->status, "RUNNING");
  EXPECT_EQ(response->retry_number, 2);
  EXPECT_EQ(response->total_retries, 3);  // Default total_retries remains unless explicitly updated
}

// Test: Active Task Pointer management using a dummy task.
// This verifies that setting and clearing the active task pointer works as expected.
TEST_F(AutowareBridgeUtilTest, ActiveTaskPointerTest) {
  auto dummy_task = std::make_shared<DummyTask>();
  util->setActiveTaskPtr(dummy_task);
  EXPECT_EQ(util->getActiveTaskPtr(), dummy_task);

  util->clearActiveTaskPtr();
  EXPECT_EQ(util->getActiveTaskPtr(), nullptr);
}

//-------------------------------------------------------------
// Concurrency and Multiple Tasks Tests
//-------------------------------------------------------------

// Test: Concurrent updates to the same task to ensure thread safety.
// Multiple threads update the same task's status and retry count.
TEST_F(AutowareBridgeUtilTest, ConcurrentUpdateTest) {
  std::string task_id = "concurrent_task";
  util->updateTaskId(task_id);
  const int num_threads = 10;
  const int num_iterations = 100;
  std::vector<std::thread> threads;

  auto updateFunc = [&]() {
    for (int i = 0; i < num_iterations; ++i) {
      util->updateTaskStatus(task_id, "RUNNING");
      util->updateTaskRetries(task_id, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  };

  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back(updateFunc);
  }

  for (auto &t : threads) {
    t.join();
  }

  TaskInfo info = util->getTaskStatus(task_id);
  // The final retry_number could be any value between 0 and num_iterations - 1.
  EXPECT_GE(info.retry_number, 0);
  EXPECT_LE(info.retry_number, num_iterations - 1);
}

// Test: Multiple tasks scenario where setting a new task clears the previous one.
// This confirms that only one active task exists at a time.
TEST_F(AutowareBridgeUtilTest, MultipleTasksTest) {
  // Set the first task.
  util->updateTaskId("task1");
  EXPECT_TRUE(util->isTaskActive("task1"));

  // Setting a new task ID should clear the previous task.
  util->updateTaskId("task2");
  EXPECT_FALSE(util->isTaskActive("task1"));
  EXPECT_TRUE(util->isTaskActive("task2"));
}

//-------------------------------------------------------------
// Additional Edge Case Tests
//-------------------------------------------------------------

// Test: Using an invalid request type (e.g., TaskRequestType::REASON).
// The updateTask function should not modify any fields when an unhandled type is passed.
TEST_F(AutowareBridgeUtilTest, InvalidRequestTypeTest) {
  std::string task_id = "task_invalid";
  util->updateTaskId(task_id);

  // Capture the current TaskInfo.
  TaskInfo original = util->getTaskStatus(task_id);

  // Use an unsupported request type (REASON) to update the task.
  util->updateTask(task_id, TaskRequestType::REASON, "SomeReason", 10);

  TaskInfo updated = util->getTaskStatus(task_id);
  // The original values should remain unchanged.
  EXPECT_EQ(updated.status, original.status);
  EXPECT_EQ(updated.retry_number, original.retry_number);
  EXPECT_EQ(updated.total_retries, original.total_retries);
  EXPECT_EQ(updated.cancel_status, original.cancel_status);
}

// Test: Handling an empty task ID.
// Verifies that the class handles an empty task ID correctly.
TEST_F(AutowareBridgeUtilTest, EmptyTaskIDTest) {
  std::string empty_id = "";
  util->updateTaskId(empty_id);
  EXPECT_EQ(util->getActiveTaskId(), empty_id);

  // Update status on the empty task ID.
  util->updateTaskStatus(empty_id, "RUNNING");
  TaskInfo info = util->getTaskStatus(empty_id);
  EXPECT_EQ(info.status, "RUNNING");
}
