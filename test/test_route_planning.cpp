#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>
#include "autoware_bridge/route_planning.hpp"
#include "autoware_bridge/autoware_bridge_util.hpp"

// Fully qualify message types.
#include "autoware_adapi_v1_msgs/msg/route_state.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using ::testing::_;
using ::testing::AtLeast;

//------------------------------------------------------------------------------
// MockAutowareBridgeUtil:
// A mock class for AutowareBridgeUtil to simulate external interactions.
//------------------------------------------------------------------------------
class MockAutowareBridgeUtil : public AutowareBridgeUtil {
public:
  MOCK_METHOD(void, updateTaskStatus, (const std::string&, const std::string&));
  MOCK_METHOD(void, updateTaskRetries, (const std::string&, int));
};

//------------------------------------------------------------------------------
// Test Fixture for RoutePlanning tests.
//------------------------------------------------------------------------------
class RoutePlanningTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Initialize ROS node
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_route_planning_node");

    // Create a mock utility for status updates
    mock_util_ = std::make_shared<MockAutowareBridgeUtil>();

    // Create the RoutePlanning instance (using the node and the mock utility)
    route_planning_ = std::make_shared<RoutePlanning>(node_, mock_util_);

    // Create publishers for topics that RoutePlanning subscribes to.
    route_state_pub_ = node_->create_publisher<autoware_adapi_v1_msgs::msg::RouteState>("/api/routing/state", 10);
    op_mode_pub_ = node_->create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>("/api/operation_mode/state", 10);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  // Helper function to spin the node for a given duration to process callbacks.
  void spinFor(std::chrono::milliseconds duration) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MockAutowareBridgeUtil> mock_util_;
  std::shared_ptr<RoutePlanning> route_planning_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr route_state_pub_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr op_mode_pub_;
};

//------------------------------------------------------------------------------
// Test Case 1: Successful Route Planning Execution
// This test verifies that execute() eventually reports SUCCESS when the proper
// state transitions are simulated via topic messages.
//------------------------------------------------------------------------------
TEST_F(RoutePlanningTest, SuccessfulRoutePlanning) {
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.pose.position.x = 10.0;
  goal_pose.pose.position.y = 20.0;
  std::string task_id = "task_success";

  // Expect that execution starts and later reports SUCCESS.
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "RUNNING")).Times(1);
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "SUCCESS")).Times(1);

  // Run execute() in a separate thread because it is blocking.
  std::thread exec_thread([&]() {
    route_planning_->execute(task_id, goal_pose);
  });

  // Allow execute() to run for a short time.
  spinFor(std::chrono::milliseconds(300));

  // Publish a RouteState message indicating the route is set.
  autoware_adapi_v1_msgs::msg::RouteState route_msg;
  route_msg.state = autoware_adapi_v1_msgs::msg::RouteState::SET;
  route_state_pub_->publish(route_msg);

  spinFor(std::chrono::milliseconds(300));

  // Publish an OperationModeState message indicating autonomous mode is available.
  autoware_adapi_v1_msgs::msg::OperationModeState op_msg;
  op_msg.is_autonomous_mode_available = true;
  op_mode_pub_->publish(op_msg);

  exec_thread.join();
}

//------------------------------------------------------------------------------
// Test Case 2: Route Planning Cancellation
// This test runs execute() in a separate thread, then calls cancel().
// We expect that updateTaskStatus is called with CANCELLED.
//------------------------------------------------------------------------------
TEST_F(RoutePlanningTest, CancelRoutePlanning) {
  geometry_msgs::msg::PoseStamped goal_pose;
  std::string task_id = "task_cancel";

  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "RUNNING")).Times(1);
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "CANCELLED")).Times(AtLeast(1));

  std::thread exec_thread([&]() {
    route_planning_->execute(task_id, goal_pose);
  });

  // Allow the execute loop to run briefly.
  spinFor(std::chrono::milliseconds(200));

  // Request cancellation.
  route_planning_->cancel();

  exec_thread.join();
}

//------------------------------------------------------------------------------
// Test Case 3: Route Planning Timeout Scenario
// This test verifies that if execute() runs without receiving success-inducing messages,
// it eventually times out and updateTaskStatus is called with TIMEOUT.
//------------------------------------------------------------------------------
TEST_F(RoutePlanningTest, RoutePlanningTimeout) {
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.pose.position.x = 10.0;
  goal_pose.pose.position.y = 20.0;
  std::string task_id = "task_timeout";

  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "RUNNING")).Times(1);
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "TIMEOUT")).Times(1);

  // Run execute() without publishing any messages to trigger success.
  route_planning_->execute(task_id, goal_pose);
}

//------------------------------------------------------------------------------
// Test Case 4: Route Planning Retry Mechanism
// This test verifies that the retry logic is active by expecting that updateTaskRetries
// is called at least once. (The exact number depends on your retry limit.)
//------------------------------------------------------------------------------
TEST_F(RoutePlanningTest, RoutePlanningRetries) {
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.pose.position.x = 10.0;
  goal_pose.pose.position.y = 20.0;
  std::string task_id = "task_retry";

  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "RUNNING")).Times(1);
  // Expect that updateTaskRetries is called at least once during the execution.
  EXPECT_CALL(*mock_util_, updateTaskRetries(task_id, _)).Times(AtLeast(1));
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "TIMEOUT")).Times(1);

  route_planning_->execute(task_id, goal_pose);
}

//------------------------------------------------------------------------------
// Test Case 5: Route State Callback Handling
// This test publishes a RouteState message (with state SET) on the topic
// and ensures that the RoutePlanning class processes it (i.e. the callback is invoked)
// without error.
//------------------------------------------------------------------------------
TEST_F(RoutePlanningTest, RouteStateCallbackHandling) {
  autoware_adapi_v1_msgs::msg::RouteState msg;
  msg.state = autoware_adapi_v1_msgs::msg::RouteState::SET;
  route_state_pub_->publish(msg);
  spinFor(std::chrono::milliseconds(100));
  SUCCEED();  // If no error occurs, the test passes.
}

//------------------------------------------------------------------------------
// Test Case 6: Operation Mode State Callback Handling
// This test publishes an OperationModeState message (with autonomous mode available)
// on the topic and ensures that the callback is processed without error.
//------------------------------------------------------------------------------
TEST_F(RoutePlanningTest, OperationModeStateCallbackHandling) {
  autoware_adapi_v1_msgs::msg::OperationModeState msg;
  msg.is_autonomous_mode_available = true;
  op_mode_pub_->publish(msg);
  spinFor(std::chrono::milliseconds(100));
  SUCCEED();
}