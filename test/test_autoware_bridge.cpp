#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ftd_master_msgs/msg/pose_stamped_with_task_id.hpp>
#include <autoware_bridge/srv/get_task_status.hpp>
#include "autoware_bridge/autoware_bridge_util.hpp"
#include <chrono>
#include <thread>

#define private public
#define protected public
#include "autoware_bridge/autoware_bridge.hpp"
#undef private
#undef protected

using namespace std::chrono_literals;

// A DummyTask that records execute() and cancel() calls
class DummyTask : public BaseTask {
public:
  DummyTask(std::shared_ptr<AutowareBridgeUtil> util)
    : executed_(false), cancelled_(false), util_(util) {}

  void execute(const std::string & task_id,
               const geometry_msgs::msg::PoseStamped &) override {
    executed_ = true;
    util_->updateTaskStatus(task_id, "SUCCESS");
  }

  void cancel() override { cancelled_ = true; }

  bool executed_;
  bool cancelled_;
  std::shared_ptr<AutowareBridgeUtil> util_;
};

// Expose protected/private members for direct testing
class AutowareBridgeNodeFriend : public AutowareBridgeNode {
public:
  AutowareBridgeNodeFriend(std::shared_ptr<AutowareBridgeUtil> util)
    : AutowareBridgeNode(util) {}

  using AutowareBridgeNode::startTaskExecution;
  using AutowareBridgeNode::startThreadExecution;
  using AutowareBridgeNode::isTaskRejected;
  using AutowareBridgeNode::publishTaskRejectionReason;
  using AutowareBridgeNode::publishTaskResponse;
  using AutowareBridgeNode::cancelTaskCallback;
  using AutowareBridgeNode::publishCancelResponse;
  using AutowareBridgeNode::onTimerCallback;
  using AutowareBridgeNode::localizationQualityCallback;
  using AutowareBridgeNode::handleStatusRequestSrvc;
};

// Test fixture
class AutowareBridgeNodeDirectTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    util_ = std::make_shared<AutowareBridgeUtil>();
    node_ = std::make_shared<AutowareBridgeNodeFriend>(util_);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::shared_ptr<AutowareBridgeUtil> util_;
  std::shared_ptr<AutowareBridgeNodeFriend> node_;
};

// 1. startTaskExecution with no active task should execute localization task
TEST_F(
  AutowareBridgeNodeDirectTest,
  startTaskExecution_WhenNoActiveTask_ShouldExecuteLocalizationTaskAndSetSuccessStatus
) {
  std::string task_id = "localization_1";
  geometry_msgs::msg::PoseStamped pose;
  auto localization_task = std::make_shared<DummyTask>(util_);

  node_->startTaskExecution(task_id, pose, localization_task);
  std::this_thread::sleep_for(50ms);

  EXPECT_TRUE(localization_task->executed_);
  EXPECT_EQ(util_->getTaskStatus(task_id).status, "SUCCESS");
}

// 2. startTaskExecution when another task active should reject route_planning
TEST_F(
  AutowareBridgeNodeDirectTest,
  startTaskExecution_WhenAnotherTaskActive_ShouldNotExecuteRoutePlanningTask
) {
  util_->updateTaskId("existing_task");
  util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));

  std::string task_id = "route_planning_1";
  geometry_msgs::msg::PoseStamped pose;
  auto route_planning_task = std::make_shared<DummyTask>(util_);

  node_->startTaskExecution(task_id, pose, route_planning_task);

  EXPECT_FALSE(route_planning_task->executed_);
  EXPECT_NE(util_->getTaskStatus(task_id).status, "SUCCESS");
}

// 3. startThreadExecution with null pointer should not crash
TEST_F(
  AutowareBridgeNodeDirectTest,
  startThreadExecution_WithNullActiveTask_ShouldLogErrorAndNotCrash
) {
  EXPECT_NO_THROW(
    node_->startThreadExecution("autonomous_driving_1", geometry_msgs::msg::PoseStamped())
  );
}

// 4. isTaskRejected should return false when no active task
TEST_F(
  AutowareBridgeNodeDirectTest,
  isTaskRejected_WhenNoActiveTask_ShouldReturnFalse
) {
  util_->clearActiveTaskPtr();
  EXPECT_FALSE(node_->isTaskRejected("localization"));
}

// 5. isTaskRejected should return true when active ID == NO_ACTIVE_TASK
TEST_F(
  AutowareBridgeNodeDirectTest,
  isTaskRejected_WhenActiveIdIsNoActiveTask_ShouldReturnTrue
) {
  util_->updateTaskId("NO_ACTIVE_TASK");
  util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));
  EXPECT_TRUE(node_->isTaskRejected("route_planning"));
}

// 6. isTaskRejected should return true when some other task is active
TEST_F(
  AutowareBridgeNodeDirectTest,
  isTaskRejected_WhenDifferentTaskActive_ShouldReturnTrue
) {
  util_->updateTaskId("busy_task");
  util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));
  EXPECT_TRUE(node_->isTaskRejected("autonomous_driving"));
}

// 7. publishTaskRejectionReason should do nothing for NO_ACTIVE_TASK
TEST_F(
  AutowareBridgeNodeDirectTest,
  publishTaskRejectionReason_WhenNoActiveTask_ShouldDoNothing
) {
  EXPECT_NO_THROW(
    node_->publishTaskRejectionReason("localization", "NO_ACTIVE_TASK")
  );
}

// 8. publishTaskRejectionReason should publish rejection for active task
TEST_F(
  AutowareBridgeNodeDirectTest,
  publishTaskRejectionReason_WhenTaskActive_ShouldPublishRejected
) {
  util_->updateTaskId("busy_task");
  EXPECT_NO_THROW(
    node_->publishTaskRejectionReason("route_planning", "busy_task")
  );
}

// 9. publishTaskResponse should publish when task is active
TEST_F(
  AutowareBridgeNodeDirectTest,
  publishTaskResponse_WhenTaskIsActive_ShouldPublishCurrentStatus
) {
  std::string task_id = "task_response";
  util_->updateTaskId(task_id);
  util_->updateTaskStatus(task_id, "IN_PROGRESS");
  util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));

  EXPECT_NO_THROW(
    node_->publishTaskResponse(task_id)
  );
}

// 10. publishTaskResponse should warn when task is not active
TEST_F(
  AutowareBridgeNodeDirectTest,
  publishTaskResponse_WhenTaskIsNotActive_ShouldWarnAndNotCrash
) {
  util_->clearActiveTaskPtr();
  EXPECT_NO_THROW(
    node_->publishTaskResponse("autonomous_driving_123")
  );
}

// 11. cancelTaskCallback should cancel and set flag for matching active task
TEST_F(
    AutowareBridgeNodeDirectTest,
    cancelTaskCallback_WhenActiveTaskMatches_ShouldInvokeCancelAndSetFlag
  ) {
    // Arrange
    std::string task_id = "cancel_me";
    
    // Create a shared pointer to a dummy task and update the task ID
    auto autonomous_driving_task = std::make_shared<DummyTask>(util_);
    util_->updateTaskId(task_id);
    util_->setActiveTaskPtr(autonomous_driving_task);
  
    // Set up the message that will be passed to the callback
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = task_id;
  
    // Act - Call the cancel task callback with the message
    node_->cancelTaskCallback(msg);
  
    // Assert - Verify that the task has been cancelled, the flag is set, and the status is updated
    EXPECT_TRUE(autonomous_driving_task->cancelled_);
    EXPECT_TRUE(node_->is_cancel_requested_);
    EXPECT_EQ(util_->getTaskStatus(task_id).status, "CANCELLED");
  }
  

// 12. cancelTaskCallback should reject when no matching active task
TEST_F(
  AutowareBridgeNodeDirectTest,
  cancelTaskCallback_WhenNoActiveTask_ShouldPublishRejection
) {
  util_->clearActiveTaskPtr();
  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data = "no_such_task";
  EXPECT_NO_THROW(
    node_->cancelTaskCallback(msg)
  );
}

// 13. publishCancelResponse should publish when task is active
TEST_F(
  AutowareBridgeNodeDirectTest,
  publishCancelResponse_WhenTaskIsActive_ShouldPublishCancelStatus
) {
  std::string task_id = "z1";
  util_->updateTaskId(task_id);
  util_->updateTaskStatus(task_id, "CANCELLED");
  util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));

  EXPECT_NO_THROW(
    node_->publishCancelResponse(task_id)
  );
}

// 14. publishCancelResponse should warn when task is not active
TEST_F(
  AutowareBridgeNodeDirectTest,
  publishCancelResponse_WhenTaskIsNotActive_ShouldWarnAndNotCrash
) {
  util_->clearActiveTaskPtr();
  EXPECT_NO_THROW(
    node_->publishCancelResponse("autonomous_driving_123")
  );
}

// 15. onTimerCallback should trigger reinit when quality is poor
TEST_F(
  AutowareBridgeNodeDirectTest,
  onTimerCallback_WhenQualityIsPoorAndLocalizationSuccess_ShouldPublishRetryingStatus
) {
  util_->updateTaskId("localization_abc");
  util_->updateTaskStatus("localization_abc", "SUCCESS");
  node_->localization_quality_ = false;

  EXPECT_NO_THROW(
    node_->onTimerCallback()
  );
  EXPECT_EQ(util_->getTaskStatus("localization_abc").status, "RETRYING");
}

// 16. onTimerCallback should do nothing when quality is good
TEST_F(
  AutowareBridgeNodeDirectTest,
  onTimerCallback_WhenQualityIsGood_ShouldDoNothing
) {
  util_->updateTaskId("localization_xyz");
  util_->updateTaskStatus("localization_xyz", "SUCCESS");
  node_->localization_quality_ = true;

  EXPECT_NO_THROW(
    node_->onTimerCallback()
  );
}

// 17. localizationQualityCallback should update the quality flag
TEST_F(
  AutowareBridgeNodeDirectTest,
  localizationQualityCallback_WhenCalled_ShouldSetQualityFlag
) {
  tier4_system_msgs::msg::ModeChangeAvailable msg;
  msg.available = true;
  node_->localizationQualityCallback(msg);
  EXPECT_TRUE(node_->localization_quality_);
}
