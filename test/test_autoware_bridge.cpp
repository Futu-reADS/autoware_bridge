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


// Minimal DummyTask for testing real task execution
class DummyTask : public BaseTask
{
public:
  DummyTask(std::shared_ptr<AutowareBridgeUtil> util)
  : executed_(false), cancelled_(false), util_(util) {}

  void execute(const std::string & task_id, const geometry_msgs::msg::PoseStamped &) override
  {
    executed_ = true;
    util_->updateTaskStatus(task_id, "SUCCESS");
  }

  void cancel() override { cancelled_ = true; }

  bool executed_;
  bool cancelled_;
  std::shared_ptr<AutowareBridgeUtil> util_;
};

// Friend wrapper class to expose private methods
class AutowareBridgeNodeFriend : public AutowareBridgeNode
{
public:
  AutowareBridgeNodeFriend(std::shared_ptr<AutowareBridgeUtil> util)
  : AutowareBridgeNode(util) {}

  using AutowareBridgeNode::startTaskExecution;
  using AutowareBridgeNode::publishTaskRejectionReason;
  using AutowareBridgeNode::publishTaskResponse;
  using AutowareBridgeNode::isTaskRejected;
};

// GTest fixture
class AutowareBridgeNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    util_ = std::make_shared<AutowareBridgeUtil>();
    node_ = std::make_shared<AutowareBridgeNodeFriend>(util_);
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<AutowareBridgeUtil> util_;
  std::shared_ptr<AutowareBridgeNodeFriend> node_;
};

// Branch: Task accepted and executed
TEST_F(AutowareBridgeNodeTest, TaskAcceptedAndExecuted)
{
  std::string task_id = "task_1";
  geometry_msgs::msg::PoseStamped pose;
  auto task = std::make_shared<DummyTask>(util_);

  node_->startTaskExecution(task_id, pose, task);

  auto status = util_->getTaskStatus(task_id);
  EXPECT_EQ(status.status, "SUCCESS");
  EXPECT_TRUE(task->executed_);
}

// Branch: Task rejected when one is already active
TEST_F(AutowareBridgeNodeTest, TaskRejectedWhenAnotherTaskIsActive)
{
  util_->updateTaskId("task_1");
  EXPECT_TRUE(node_->isTaskRejected("localization"));
}

// Branch: Task rejection reason - no active task
TEST_F(AutowareBridgeNodeTest, TaskRejectionReasonNoActiveTask)
{
  EXPECT_NO_THROW({
    node_->publishTaskRejectionReason("localization", "NO_ACTIVE_TASK");
  });
}

// Branch: Task rejection reason published
TEST_F(AutowareBridgeNodeTest, TaskRejectionReasonPublished)
{
  util_->updateTaskId("active_task");
  EXPECT_NO_THROW({
    node_->publishTaskRejectionReason("route_planning", "active_task");
  });
}

// Branch: Publish task response when active
TEST_F(AutowareBridgeNodeTest, PublishTaskResponseWhenActive)
{
  std::string task_id = "task_2";
  util_->updateTaskId(task_id);
  util_->updateTaskStatus(task_id, "IN_PROGRESS");

  EXPECT_NO_THROW({
    node_->publishTaskResponse(task_id);
  });
}

// Branch: Update and get task status correctly
TEST_F(AutowareBridgeNodeTest, UpdateAndGetTaskStatus)
{
  std::string task_id = "task_3";
  util_->updateTaskId(task_id);
  util_->updateTaskStatus(task_id, "PENDING");

  auto status = util_->getTaskStatus(task_id);
  EXPECT_EQ(status.status, "PENDING");
  EXPECT_EQ(status.retry_number, 0);
}

// Branch: Retry tracking works
TEST_F(AutowareBridgeNodeTest, TaskRetryTracking)
{
  std::string task_id = "task_retry";
  util_->updateTaskId(task_id);
  util_->updateTaskRetries(task_id, 1);

  auto status = util_->getTaskStatus(task_id);
  EXPECT_EQ(status.retry_number, 1);
}

// Branch: Active task pointer handling
TEST_F(AutowareBridgeNodeTest, ActiveTaskPtrHandling)
{
  EXPECT_EQ(util_->getActiveTaskPtr(), nullptr);

  auto task = std::make_shared<DummyTask>(util_);
  util_->setActiveTaskPtr(task);
  EXPECT_EQ(util_->getActiveTaskPtr(), task);

  util_->clearActiveTaskPtr();
  EXPECT_EQ(util_->getActiveTaskPtr(), nullptr);
}
