#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ftd_master_msgs/msg/pose_stamped_with_task_id.hpp>
#include <tier4_system_msgs/msg/mode_change_available.hpp>
#include <autoware_bridge/srv/get_task_status.hpp>
#include <autoware_bridge/autoware_bridge_util.hpp>

// Expose private methods and members
#define private public
#define protected public
#include <autoware_bridge/autoware_bridge.hpp>
#undef private
#undef protected

#include <chrono>
#include <thread>
#include <memory>

using namespace std::chrono_literals;

// DummyTask: stub for BaseTask
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
  bool executed_ = false;
  bool cancelled_ = false;
  std::shared_ptr<AutowareBridgeUtil> util_;
};

// Stub tasks to avoid name collision with real classes
class StubLocalization : public DummyTask {
public:
  StubLocalization(std::shared_ptr<AutowareBridgeNode>, std::shared_ptr<AutowareBridgeUtil> util)
    : DummyTask(util) {}
};
class StubRoutePlanning : public DummyTask {
public:
  StubRoutePlanning(std::shared_ptr<AutowareBridgeNode>, std::shared_ptr<AutowareBridgeUtil> util)
    : DummyTask(util) {}
};
class StubAutonomousDriving : public DummyTask {
public:
  StubAutonomousDriving(std::shared_ptr<AutowareBridgeNode>, std::shared_ptr<AutowareBridgeUtil> util)
    : DummyTask(util) {}
};

// SlowTask for cancellation path
class SlowTask : public BaseTask {
public:
  SlowTask(std::shared_ptr<AutowareBridgeUtil> util) : cancelled_(false), util_(util) {}
  void execute(const std::string & task_id, const geometry_msgs::msg::PoseStamped &) override {
    std::this_thread::sleep_for(100ms);
    util_->updateTaskStatus(task_id, "RUNNING");
  }
  void cancel() override { cancelled_ = true; }
  bool cancelled_ = false;
  std::shared_ptr<AutowareBridgeUtil> util_;
};

// Test fixture
class AutowareBridgeNodeTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    util_ = std::make_shared<AutowareBridgeUtil>();
    node_ = std::make_shared<AutowareBridgeNode>(util_);
  }
  void TearDown() override {
    if (rclcpp::ok()) rclcpp::shutdown();
  }
  std::shared_ptr<AutowareBridgeUtil> util_;
  std::shared_ptr<AutowareBridgeNode> node_;
};

// 1. startTaskExecution: no active
TEST_F(AutowareBridgeNodeTest, StartTaskExecution_NoActive_ExecutesAndSetsSuccess) {
  auto t = std::make_shared<DummyTask>(util_);
  node_->startTaskExecution("loc1", geometry_msgs::msg::PoseStamped(), t);
  std::this_thread::sleep_for(50ms);
  EXPECT_TRUE(t->executed_);
  EXPECT_EQ(util_->getTaskStatus("loc1").status, "SUCCESS");
}

// 2. startTaskExecution: active exists
TEST_F(AutowareBridgeNodeTest, StartTaskExecution_ActiveExists_DoesNotExecuteNew) {
  util_->updateTaskId("busy");
  util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));
  auto t = std::make_shared<DummyTask>(util_);
  node_->startTaskExecution("route1", geometry_msgs::msg::PoseStamped(), t);
  EXPECT_FALSE(t->executed_);
}

// 3. startThreadExecution: no active ptr
TEST_F(AutowareBridgeNodeTest, StartThreadExecution_NoActive_NoCrash) {
  EXPECT_NO_THROW(node_->startThreadExecution("auto1", geometry_msgs::msg::PoseStamped()));
}

// 4. isTaskRejected: no active
TEST_F(AutowareBridgeNodeTest, IsTaskRejected_NoActive_ReturnsFalse) {
  util_->clearActiveTaskPtr();
  EXPECT_FALSE(node_->isTaskRejected("localization"));
}

// 5. isTaskRejected: NO_ACTIVE_TASK ID
TEST_F(AutowareBridgeNodeTest, IsTaskRejected_NoActiveID_ReturnsTrue) {
  util_->updateTaskId("NO_ACTIVE_TASK");
  util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));
  EXPECT_TRUE(node_->isTaskRejected("route_planning"));
}

// 6. isTaskRejected: other active
TEST_F(AutowareBridgeNodeTest, IsTaskRejected_OtherActive_ReturnsTrue) {
  util_->updateTaskId("xyz");
  util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));
  EXPECT_TRUE(node_->isTaskRejected("autonomous_driving"));
}

// 7. publishTaskRejectionReason: NO_ACTIVE_TASK no crash
TEST_F(AutowareBridgeNodeTest, PublishRejectionReason_NoActive_DoesNothing) {
  EXPECT_NO_THROW(node_->publishTaskRejectionReason("loc","NO_ACTIVE_TASK"));
}

// 8. publishTaskRejectionReason: publishes
TEST_F(AutowareBridgeNodeTest, PublishRejectionReason_Active_Publishes) {
  util_->updateTaskId("busy");
  EXPECT_NO_THROW(node_->publishTaskRejectionReason("route","busy"));
}

// 9. publishTaskResponse: active publishes
TEST_F(AutowareBridgeNodeTest, PublishTaskResponse_Active_Publishes) {
  util_->updateTaskId("t1");
  util_->updateTaskStatus("t1","DONE");
  util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));
  EXPECT_NO_THROW(node_->publishTaskResponse("t1"));
}

// 10. publishTaskResponse: inactive no crash
TEST_F(AutowareBridgeNodeTest, PublishTaskResponse_Inactive_NoCrash) {
  util_->clearActiveTaskPtr();
  EXPECT_NO_THROW(node_->publishTaskResponse("none"));
}

// 11. cancelTaskCallback: matching
TEST_F(AutowareBridgeNodeTest, CancelTaskCallback_Match_Cancels) {
  auto t = std::make_shared<DummyTask>(util_);
  util_->updateTaskId("cancel");
  util_->setActiveTaskPtr(t);
  auto msg = std::make_shared<std_msgs::msg::String>(); msg->data = "cancel";
  node_->cancelTaskCallback(msg);
  EXPECT_TRUE(t->cancelled_);
  EXPECT_TRUE(node_->is_cancel_requested_);
}

// 12. cancelTaskCallback: no match
TEST_F(AutowareBridgeNodeTest, CancelTaskCallback_NoMatch_NoCrash) {
  util_->clearActiveTaskPtr();
  auto msg = std::make_shared<std_msgs::msg::String>(); msg->data = "foo";
  EXPECT_NO_THROW(node_->cancelTaskCallback(msg));
}

// 13. publishCancelResponse: active
TEST_F(AutowareBridgeNodeTest, PublishCancelResponse_Active_Publishes) {
  util_->updateTaskId("c1"); util_->updateTaskStatus("c1","DONE");
  util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));
  EXPECT_NO_THROW(node_->publishCancelResponse("c1"));
}

// 14. publishCancelResponse: inactive
TEST_F(AutowareBridgeNodeTest, PublishCancelResponse_Inactive_NoCrash) {
  util_->clearActiveTaskPtr();
  EXPECT_NO_THROW(node_->publishCancelResponse("none"));
}

// 15. onTimerCallback: poor localization
TEST_F(AutowareBridgeNodeTest, OnTimer_PoorLocalization_Reinit) {
  util_->updateTaskId("lq"); util_->updateTaskStatus("lq","SUCCESS");
  node_->localization_quality_ = false;
  EXPECT_NO_THROW(node_->onTimerCallback());
}

// 16. onTimerCallback: good localization
TEST_F(AutowareBridgeNodeTest, OnTimer_GoodLocalization_NoCrash) {
  util_->updateTaskId("lq"); util_->updateTaskStatus("lq","SUCCESS");
  node_->localization_quality_ = true;
  EXPECT_NO_THROW(node_->onTimerCallback());
}

// 17. localizationQualityCallback
TEST_F(AutowareBridgeNodeTest, LocalizationQualityCallback_SetsFlag) {
  tier4_system_msgs::msg::ModeChangeAvailable m; m.available = true;
  node_->localizationQualityCallback(m);
  EXPECT_TRUE(node_->localization_quality_);
}

// 18. localizationRequestCallback: no active
TEST_F(AutowareBridgeNodeTest, LocalizationRequest_NoActive_SetsId) {
  auto req = std::make_shared<ftd_master_msgs::msg::PoseStampedWithTaskId>();
  req->task_id.data = "locr";
  node_->localizationRequestCallback(req);
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(util_->getActiveTaskId(), "locr");
}

// 19. routePlanningRequestCallback: no active
TEST_F(AutowareBridgeNodeTest, RoutePlanningRequest_NoActive_SetsId) {
  auto req = std::make_shared<ftd_master_msgs::msg::PoseStampedWithTaskId>();
  req->task_id.data = "rpr";
  node_->routePlanningRequestCallback(req);
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(util_->getActiveTaskId(), "rpr");
}

// 20. autonomousDrivingRequestCallback: no active
TEST_F(AutowareBridgeNodeTest, AutonomousDrivingRequest_NoActive_SetsId) {
  auto req = std::make_shared<std_msgs::msg::String>(); req->data = "adr";
  node_->autonomousDrivingRequestCallback(req);
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(util_->getActiveTaskId(), "adr");
}

// 21. startThreadExecution: with active executes and clears ptr
TEST_F(AutowareBridgeNodeTest, StartThreadExecution_WithActive_Executes) {
  auto t = std::make_shared<DummyTask>(util_);
  util_->updateTaskId("th"); util_->setActiveTaskPtr(t);
  node_->startThreadExecution("th", geometry_msgs::msg::PoseStamped());
  std::this_thread::sleep_for(50ms);
  EXPECT_TRUE(t->executed_);
  EXPECT_FALSE(util_->getActiveTaskPtr());
}

// 22. handleStatusRequestSrvc
TEST_F(AutowareBridgeNodeTest, HandleStatusRequestSrvc_Populates) {
  util_->updateTaskId("st"); util_->updateTaskStatus("st","DONE");
  auto req = std::make_shared<autoware_bridge::srv::GetTaskStatus::Request>(); req->task_id = "st";
  auto res = std::make_shared<autoware_bridge::srv::GetTaskStatus::Response>();
  node_->handleStatusRequestSrvc(req, res);
  EXPECT_EQ(res->status, "DONE");
}

// 23. onTimerCallback: route planning poor triggers reinit
TEST_F(AutowareBridgeNodeTest, OnTimer_RoutePlanningPoor_Reinit) {
  util_->updateTaskId("rp"); util_->updateTaskStatus("rp","RUNNING");
  node_->localization_quality_ = false;
  EXPECT_NO_THROW(node_->onTimerCallback());
}

// 24. onTimerCallback: autonomous driving poor triggers reinit
TEST_F(AutowareBridgeNodeTest, OnTimer_AutoDrivingPoor_Reinit) {
  util_->updateTaskId("ad"); util_->updateTaskStatus("ad","RUNNING");
  node_->localization_quality_ = false;
  EXPECT_NO_THROW(node_->onTimerCallback());
}

// 25. startThreadExecution: cancellation path
TEST_F(AutowareBridgeNodeTest, StartThreadExecution_CancellationPath_Cancels) {
  auto slow = std::make_shared<SlowTask>(util_);
  util_->updateTaskId("slow"); util_->setActiveTaskPtr(slow);
  node_->startThreadExecution("slow", geometry_msgs::msg::PoseStamped());
  std::this_thread::sleep_for(10ms);
  auto msg = std::make_shared<std_msgs::msg::String>(); msg->data = "slow";
  node_->cancelTaskCallback(msg);
  std::this_thread::sleep_for(200ms);
  EXPECT_TRUE(slow->cancelled_);
}

// 26. localizationRequestCallback busy ignores
TEST_F(AutowareBridgeNodeTest, LocalizationRequest_Busy_Ignores) {
  util_->updateTaskId("lb"); util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));
  auto req = std::make_shared<ftd_master_msgs::msg::PoseStampedWithTaskId>(); req->task_id.data = "ignored";
  node_->localizationRequestCallback(req);
  EXPECT_EQ(util_->getActiveTaskId(), "lb");
}

// 27. routePlanningRequestCallback busy ignores
TEST_F(AutowareBridgeNodeTest, RoutePlanningRequest_Busy_Ignores) {
  util_->updateTaskId("rb"); util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));
  auto req = std::make_shared<ftd_master_msgs::msg::PoseStampedWithTaskId>(); req->task_id.data = "ignored";
  node_->routePlanningRequestCallback(req);
  EXPECT_EQ(util_->getActiveTaskId(), "rb");
}

// 28. autonomousDrivingRequestCallback busy ignores
TEST_F(AutowareBridgeNodeTest, AutonomousDrivingRequest_Busy_Ignores) {
  util_->updateTaskId("ab"); util_->setActiveTaskPtr(std::make_shared<DummyTask>(util_));
  auto req = std::make_shared<std_msgs::msg::String>(); req->data = "ignored";
  node_->autonomousDrivingRequestCallback(req);
  EXPECT_EQ(util_->getActiveTaskId(), "ab");
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
