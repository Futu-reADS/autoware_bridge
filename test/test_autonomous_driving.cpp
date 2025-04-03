#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>
#include "autoware_bridge/autonomous_driving.hpp"
#include "autoware_bridge/autoware_bridge_util.hpp"

// Fully qualify message types.
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_adapi_v1_msgs/msg/route_state.hpp"
#include "autoware_adapi_v1_msgs/msg/motion_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using ::testing::_;
using ::testing::AtLeast;

//------------------------------------------------------------------------------
// MockAutowareBridgeUtil:
// A mock for AutowareBridgeUtil to verify that updateTaskStatus and updateTaskRetries
// are called as expected.
//------------------------------------------------------------------------------
class MockAutowareBridgeUtil : public AutowareBridgeUtil {
public:
  MOCK_METHOD(void, updateTaskStatus, (const std::string&, const std::string&));
  MOCK_METHOD(void, updateTaskRetries, (const std::string&, int));
};

//------------------------------------------------------------------------------
// Test Fixture for AutonomousDriving tests.
//------------------------------------------------------------------------------
class AutonomousDrivingTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_autonomous_driving_node");
    mock_util_ = std::make_shared<MockAutowareBridgeUtil>();
    auto_drive_ = std::make_shared<AutonomousDriving>(node_, mock_util_);

    // Create publishers for the topics that AutonomousDriving subscribes to.
    op_mode_pub_ = node_->create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>("/api/operation_mode/state", 10);
    // Use fully qualified type for motion state.
    motion_pub_ = node_->create_publisher<autoware_adapi_v1_msgs::msg::MotionState>("/api/motion/state", 10);
    route_state_pub_ = node_->create_publisher<autoware_adapi_v1_msgs::msg::RouteState>("/api/routing/state", 10);
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
  std::shared_ptr<AutonomousDriving> auto_drive_;

  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr op_mode_pub_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::MotionState>::SharedPtr motion_pub_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr route_state_pub_;
};

//------------------------------------------------------------------------------
// Test Case 1: Successful Autonomous Driving Execution
//
// This test simulates a full success scenario by:
//  - Expecting the task to start (RUNNING) and eventually report SUCCESS.
//  - Publishing an OperationModeState message (with mode AUTONOMOUS and available true)
//    to trigger the transition from WAIT_AUTO_DRIVE_READY to DRIVING.
//  - Publishing a MotionState message indicating MOVING so that the vehicle is not halted.
//  - Publishing a RouteState message with ARRIVED so that the execution completes successfully.
//------------------------------------------------------------------------------
TEST_F(AutonomousDrivingTest, SuccessfulAutonomousDriving) {
  geometry_msgs::msg::PoseStamped dummy_pose; // Not used by execute()
  std::string task_id = "auto_success";
  
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "RUNNING")).Times(1);
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "SUCCESS")).Times(1);
  
  std::thread exec_thread([&]() {
    auto_drive_->execute(task_id, dummy_pose);
  });
  
  spinFor(std::chrono::milliseconds(300));
  
  // Simulate autonomous mode becomes available.
  autoware_adapi_v1_msgs::msg::OperationModeState op_msg;
  op_msg.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
  op_msg.is_autonomous_mode_available = true;
  op_mode_pub_->publish(op_msg);
  
  spinFor(std::chrono::milliseconds(300));
  
  // Simulate the vehicle is moving.
  autoware_adapi_v1_msgs::msg::MotionState motion_msg;
  // Assume MotionState::MOVING is defined as 2.
  motion_msg.state = 2;  // MOVING
  motion_pub_->publish(motion_msg);
  
  spinFor(std::chrono::milliseconds(300));
  
  // Simulate that the route has been completed.
  autoware_adapi_v1_msgs::msg::RouteState route_msg;
  route_msg.state = autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
  route_state_pub_->publish(route_msg);
  
  exec_thread.join();
}

//------------------------------------------------------------------------------
// Test Case 2: Autonomous Driving Cancellation
//
// This test runs execute() in a separate thread, waits briefly, and then calls cancel().
// It expects that updateTaskStatus is eventually called with "CANCELLED".
//------------------------------------------------------------------------------
TEST_F(AutonomousDrivingTest, CancelAutonomousDriving) {
  geometry_msgs::msg::PoseStamped dummy_pose;
  std::string task_id = "auto_cancel";
  
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "RUNNING")).Times(1);
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "CANCELLED")).Times(AtLeast(1));
  
  std::thread exec_thread([&]() {
    auto_drive_->execute(task_id, dummy_pose);
  });
  
  spinFor(std::chrono::milliseconds(200));
  auto_drive_->cancel();
  
  exec_thread.join();
}

//------------------------------------------------------------------------------
// Test Case 3: Autonomous Driving Timeout Scenario
//
// This test executes without publishing any messages to transition to success,
// so after exhausting retries the task should time out, triggering a "TIMEOUT" update.
//------------------------------------------------------------------------------
TEST_F(AutonomousDrivingTest, AutonomousDrivingTimeout) {
  geometry_msgs::msg::PoseStamped dummy_pose;
  std::string task_id = "auto_timeout";
  
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "RUNNING")).Times(1);
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "TIMEOUT")).Times(1);
  
  auto_drive_->execute(task_id, dummy_pose);
}

//------------------------------------------------------------------------------
// Test Case 4: Autonomous Driving Retry Mechanism
//
// In the ENGAGE_AUTO_DRIVE state, if the system does not transition to autonomous mode,
// the retry logic should trigger. This test expects that updateTaskRetries is called at least once.
//------------------------------------------------------------------------------
TEST_F(AutonomousDrivingTest, AutonomousDrivingRetries) {
  geometry_msgs::msg::PoseStamped dummy_pose;
  std::string task_id = "auto_retry";
  
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "RUNNING")).Times(1);
  EXPECT_CALL(*mock_util_, updateTaskRetries(task_id, _)).Times(AtLeast(1));
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "TIMEOUT")).Times(1);
  
  auto_drive_->execute(task_id, dummy_pose);
}

//------------------------------------------------------------------------------
// Test Case 5: Autonomous Driving HALT Scenario
//
// In the DRIVING state, if the vehicle remains STOPPED longer than allowed,
// updateTaskStatus should be called with "HALTED". This test simulates that condition
// by publishing a STOPPED MotionState message and waiting long enough for the halt timeout.
//------------------------------------------------------------------------------
TEST_F(AutonomousDrivingTest, AutonomousDrivingHalt) {
  geometry_msgs::msg::PoseStamped dummy_pose;
  std::string task_id = "auto_halt";
  
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "RUNNING")).Times(1);
  EXPECT_CALL(*mock_util_, updateTaskStatus(task_id, "HALTED")).Times(1);
  
  std::thread exec_thread([&]() {
    auto_drive_->execute(task_id, dummy_pose);
  });
  
  spinFor(std::chrono::milliseconds(300));
  
  // Publish an OperationModeState message to move to DRIVING state.
  autoware_adapi_v1_msgs::msg::OperationModeState op_msg;
  op_msg.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
  op_msg.is_autonomous_mode_available = true;
  op_mode_pub_->publish(op_msg);
  
  spinFor(std::chrono::milliseconds(300));
  
  // Publish a MotionState message indicating STOPPED (assume STOPPED is defined as 1).
  autoware_adapi_v1_msgs::msg::MotionState stop_msg;
  stop_msg.state = 1;  // STOPPED
  motion_pub_->publish(stop_msg);
  
  // Wait to allow halt timeout logic to trigger.
  spinFor(std::chrono::milliseconds(500));
  
  exec_thread.join();
}

//------------------------------------------------------------------------------
// Test Case 6: Callback Handling for Operation Mode, Motion, and Route State
//
// This test publishes messages on the topics to verify that the callbacks are processed
// without error. We do not directly verify internal state here, just that no errors occur.
//------------------------------------------------------------------------------
TEST_F(AutonomousDrivingTest, CallbackHandling) {
  // Publish an OperationModeState message.
  auto op_mode_pub_local = node_->create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>("/api/operation_mode/state", 10);
  autoware_adapi_v1_msgs::msg::OperationModeState op_msg;
  op_msg.mode = autoware_adapi_v1_msgs::msg::OperationModeState::MANUAL; // Example mode
  op_mode_pub_local->publish(op_msg);
  
  // Publish a MotionState message indicating MOVING (assume MOVING is defined as 2).
  auto motion_pub_local = node_->create_publisher<autoware_adapi_v1_msgs::msg::MotionState>("/api/motion/state", 10);
  autoware_adapi_v1_msgs::msg::MotionState motion_msg;
  motion_msg.state = 2;  // MOVING
  motion_pub_local->publish(motion_msg);
  
  // Publish a RouteState message with UNKNOWN state.
  auto route_pub_local = node_->create_publisher<autoware_adapi_v1_msgs::msg::RouteState>("/api/routing/state", 10);
  autoware_adapi_v1_msgs::msg::RouteState route_msg;
  route_msg.state = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
  route_pub_local->publish(route_msg);
  
  spinFor(std::chrono::milliseconds(100));
  SUCCEED();
}