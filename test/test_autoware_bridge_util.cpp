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

// -----------------------------
// Topic-Based Execution Responses
// -----------------------------

//1. Test: Update Task Status and Cancellation for "PENDING" then "CANCELLED"
TEST_F(AutowareBridgeUtilTest, UpdateTaskStatusTest) {
  std::string task_id = "test_task";
  
  // Create the task entry.
  util->updateTaskId(task_id);
  
  // Update main status to "PENDING"
  util->updateTaskStatus(task_id, "PENDING");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "PENDING");
  
  // Update retry number.
  util->updateTaskRetries(task_id, 3);
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.retry_number, 3);

  // Update total retries.
  util->updateTask(task_id, TaskRequestType::TOTAL_RETRIES, "", 5);
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.total_retries, 5);

  // Update cancellation:
  // For status "CANCELLED", updateTaskStatus should update cancellation info.
  util->updateTaskStatus(task_id, "CANCELLED");
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "CANCELLED");
  // Verify cancel status is updated to "CANCELLED"
  EXPECT_EQ(info.cancel_status, "CANCELLED");
}

//2. Test: Update Fail Status - "FAILED"
TEST_F(AutowareBridgeUtilTest, UpdateFailStatusTest) {
  std::string task_id = "fail_task";
  util->updateTaskId(task_id);
  util->updateTaskStatus(task_id, "FAILED");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "FAILED");
  // For FAILED, cancellation auto-updates to "FAILED_DUE_TO_TASK_FAILURE".
  EXPECT_EQ(info.cancel_status, "FAILED_DUE_TO_TASK_FAILURE");
}

//3. Test: Update Success Status - "SUCCESS"
TEST_F(AutowareBridgeUtilTest, UpdateSuccessStatusTest) {
  std::string task_id = "success_task";
  util->updateTaskId(task_id);
  util->updateTaskStatus(task_id, "SUCCESS");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "SUCCESS");
  // For SUCCESS, cancellation auto-updates to "FAILED_DUE_TO_TASK_SUCCESS".
  EXPECT_EQ(info.cancel_status, "FAILED_DUE_TO_TASK_SUCCESS");
}

//4. Test: Update Halt Status - "HALTED"
TEST_F(AutowareBridgeUtilTest, UpdateHaltStatusTest) {
  std::string task_id = "halt_task";
  util->updateTaskId(task_id);
  util->updateTaskStatus(task_id, "HALTED");
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "HALTED");
  // For HALTED, no cancellation update is performed so default is empty.
  EXPECT_EQ(info.cancel_status, "");
}

// -----------------------------
// Service-Based Execution Inquiry Responses
// -----------------------------

//5. Test: Handle Status Request Service - first with no active task then with active task.
TEST_F(AutowareBridgeUtilTest, HandleStatusRequestTest) {
  auto request = std::make_shared<autoware_bridge::srv::GetTaskStatus::Request>();
  auto response = std::make_shared<autoware_bridge::srv::GetTaskStatus::Response>();
  request->task_id = "service_task";

  // With no active task, expect rejection.
  util->handleStatusRequestSrvc(request, response);
  EXPECT_EQ(response->status, "REJECTED");
  EXPECT_EQ(response->retry_number, 0);
  EXPECT_EQ(response->total_retries, 0);

  // Create and update a task.
  util->updateTaskId(request->task_id);
  util->updateTaskStatus(request->task_id, "RUNNING");
  util->updateTaskRetries(request->task_id, 2);
  util->updateTask(request->task_id, TaskRequestType::TOTAL_RETRIES, "", 5);

  util->handleStatusRequestSrvc(request, response);
  EXPECT_EQ(response->status, "RUNNING");
  EXPECT_EQ(response->retry_number, 2);
  EXPECT_EQ(response->total_retries, 5);
}

// -----------------------------
// Cancellation Responses (Indirectly Tested via Status Updates)
// -----------------------------
// The cancel response conditions are verified in the above tests:
//   - "FAILED" -> "FAILED_DUE_TO_TASK_FAILURE"
//   - "SUCCESS" -> "FAILED_DUE_TO_TASK_SUCCESS"
//   - "CANCELLED" -> "CANCELLED"

// -----------------------------
// Utility Behavior: Active Task, Concurrency, and Defaults
// -----------------------------

//6. Test: Check if Task is Active.
TEST_F(AutowareBridgeUtilTest, IsTaskActiveTest) {
  std::string task_id = "active_task";
  EXPECT_FALSE(util->isTaskActive(task_id));
  
  util->updateTaskId(task_id);
  util->updateTaskStatus(task_id, "PENDING");
  EXPECT_TRUE(util->isTaskActive(task_id));
}

//7. Test: Retrieve Active Task ID.
TEST_F(AutowareBridgeUtilTest, GetActiveTaskIdTest) {
  EXPECT_EQ(util->getActiveTaskId(), "NO_ACTIVE_TASK");
  
  std::string task_id = "active_task";
  util->updateTaskId(task_id);
  EXPECT_EQ(util->getActiveTaskId(), task_id);
}

//8. Test: Retrieve Task Status
TEST_F(AutowareBridgeUtilTest, GetTaskStatusTest) {
  std::string task_id = "status_task";
  // When task_id is not updated, the default TaskInfo is returned.
  // The default status is "PENDING" (as initialized by TaskInfo(3)).
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "PENDING");
  
  util->updateTaskId(task_id);
  util->updateTaskStatus(task_id, "PENDING");
  info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "PENDING");
}

//9. Test: Active Task Pointer Management.
TEST_F(AutowareBridgeUtilTest, ActiveTaskPointerTest) {
  auto dummy = std::make_shared<DummyTask>();
  util->setActiveTaskPtr(dummy);
  EXPECT_EQ(util->getActiveTaskPtr(), dummy);
  util->clearActiveTaskPtr();
  EXPECT_EQ(util->getActiveTaskPtr(), nullptr);
}

//10. Test: Concurrent Update / Thread Safety.
TEST_F(AutowareBridgeUtilTest, ConcurrentUpdateTest) {
  std::string task_id = "concurrent_task";
  const int num_threads = 10;
  const int num_iterations = 100;
  std::vector<std::thread> threads;

  util->updateTaskId(task_id);

  auto updateFunc = [this, &task_id, num_iterations]() {
    for (int i = 0; i < num_iterations; ++i) {
      util->updateTaskStatus(task_id, "RUNNING");
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

//11. Test: Multiple Tasks Scenario (only one active task allowed).
TEST_F(AutowareBridgeUtilTest, MultipleTasksTest) {
  // Add first task.
  util->updateTaskId("task1");
  EXPECT_TRUE(util->isTaskActive("task1"));

  // Adding a second task clears previous tasks.
  util->updateTaskId("task2");
  EXPECT_FALSE(util->isTaskActive("task1"));
  EXPECT_TRUE(util->isTaskActive("task2"));
}

//12. Test: Get Task Status for a Non-Existent Task.
TEST_F(AutowareBridgeUtilTest, GetTaskStatusNonExistentTest) {
  std::string task_id = "non_existent_task";
  // When a non-existent task is queried, the default TaskInfo is returned.
  // The default status is "PENDING".
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "PENDING");
}

// -----------------------------
// Specific Test for updateTaskId Behavior
// -----------------------------

//13. Test: Verify updateTaskId clears previous tasks and initializes new task correctly.
TEST_F(AutowareBridgeUtilTest, UpdateTaskIdTest) {
  // Create a task with task_id "old_task"
  util->updateTaskId("old_task");
  EXPECT_TRUE(util->isTaskActive("old_task"));

  // Now call updateTaskId with a new task_id, which should clear previous tasks.
  util->updateTaskId("new_task");
  EXPECT_FALSE(util->isTaskActive("old_task"));
  EXPECT_TRUE(util->isTaskActive("new_task"));

  // Verify that the new task is initialized with default values.
  // The default TaskInfo(3) initializes status to "PENDING" and total_retries to 3.
  TaskInfo info = util->getTaskStatus("new_task");
  EXPECT_EQ(info.status, "PENDING");
  EXPECT_EQ(info.total_retries, 3);
}

//14. Test: Invalid request type → hits the `default` switch‑case branch
TEST_F(AutowareBridgeUtilTest, UpdateTaskWithInvalidRequestType) {
  std::string task_id = "invalid_req";
  util->updateTaskId(task_id);

  // Force an out‑of‑range enum; enters the `default:` in updateTask()
  util->updateTask(task_id,
                   static_cast<TaskRequestType>(-1),
                   /*value=*/"", /*number=*/0);

  // Nothing should crash, and status stays at its initialized “PENDING”
  TaskInfo info = util->getTaskStatus(task_id);
  EXPECT_EQ(info.status, "PENDING");
}

//15. Test: Non‑active task_id → hits the `else { RCLCPP_ERROR… }` branch
TEST_F(AutowareBridgeUtilTest, UpdateTaskNotActiveTaskErrorBranch) {
  // Never called updateTaskId(), so map is empty
  const std::string missing_id = "no_such_task";

  // This should take the “not found” path
  util->updateTask(missing_id,
                   TaskRequestType::STATUS,
                   /*value=*/"SHOULD_NOT_MATTER",
                   /*number=*/0);

  // Querying any non‑existent task still returns the default TaskInfo
  TaskInfo info = util->getTaskStatus(missing_id);
  EXPECT_EQ(info.status, "PENDING");
}
