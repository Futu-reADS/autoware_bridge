#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/motion_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unordered_map>

using namespace std::chrono_literals;

// Apply macro hack ONLY when including autonomous_driving.hpp
#define private public
#define protected public
#include "autoware_bridge/autonomous_driving.hpp"
#undef private
#undef protected

//
// TestableAutonomousDriving: Subclass of AutonomousDriving that exposes internal state
// and helper functions to simulate conditions (like timeouts) and force internal state changes.
//
class TestableAutonomousDriving : public AutonomousDriving {
public:
  using AutonomousDriving::AutonomousDriving;  // Inherit constructors

  // Helpers to simulate timeouts
  void simulateDriveWaitTimeout() {
    driving_start_time_ = node_->get_clock()->now() - rclcpp::Duration::from_seconds(DRIVE_WAIT_TIMEOUT_S + 0.5);
  }
  void simulateHaltTimeout() {
    halt_start_time_ = node_->get_clock()->now() - rclcpp::Duration::from_seconds(MAX_EGO_HALT_TIME + 0.5);
  }

  // Force-set the internal state
  void forceSetState(AutonomousDrivingTaskState new_state) {
    state_ = new_state;
  }
  void forceSetOperationModeState(const OperationModeState & op_mode) {
    operation_mode_state_ = op_mode;
  }
  void forceSetVehicleMotionState(const MotionState & motion_state) {
    vehicle_motion_state_ = motion_state.state;
  }
  void forceSetRouteState(const RouteState & route_state) {
    route_state_ = route_state.state;
  }
};

//
// Test fixture for AutonomousDriving tests
//
class AutonomousDrivingTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("autonomous_driving_test_node");

    // === STUB OUT ROS SERVICES ===
    fake_autonomous_svc_ = node->create_service<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
      "/api/operation_mode/change_to_autonomous",
      [](const std::shared_ptr<rmw_request_id_t> /*req_header*/,
         const std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request> /*req*/,
         std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response> /*res*/)
      {
        // no-op
      });
    fake_clear_route_svc_ = node->create_service<autoware_adapi_v1_msgs::srv::ClearRoute>(
      "/api/routing/clear_route",
      [](const std::shared_ptr<rmw_request_id_t> /*req_header*/,
         const std::shared_ptr<autoware_adapi_v1_msgs::srv::ClearRoute::Request> /*req*/,
         std::shared_ptr<autoware_adapi_v1_msgs::srv::ClearRoute::Response> /*res*/)
      {
        // no-op
      });

    util_ = std::make_shared<AutowareBridgeUtil>();
    autonomous_driving_ = std::make_shared<TestableAutonomousDriving>(node, util_);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  void spinFor(double seconds) {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(seconds)) {
      exec.spin_some(); std::this_thread::sleep_for(10ms);
    }
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Service<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr fake_autonomous_svc_;
  rclcpp::Service<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr fake_clear_route_svc_;
  std::shared_ptr<AutowareBridgeUtil> util_;
  std::shared_ptr<TestableAutonomousDriving> autonomous_driving_;
};

// Test 1: Cancellation Branch
TEST_F(AutonomousDrivingTest, TestAutonomousDrivingCancel) {
  std::string task_id = "autonomous_driving";
  geometry_msgs::msg::PoseStamped pose;

  util_->updateTaskId(task_id);
  util_->setActiveTaskPtr(autonomous_driving_);

  std::thread t([&]{ autonomous_driving_->execute(task_id, pose); });
  std::this_thread::sleep_for(200ms);
  autonomous_driving_->cancel();
  t.join();

  EXPECT_EQ(util_->getTaskStatus(task_id).status, "CANCELLED");
}

// Test 2: Timeout After Maximum Drive Retries
TEST_F(AutonomousDrivingTest, TestAutonomousDrivingTimeoutAfterRetries) {
  std::string task_id = "autonomous_drive_timeout";
  geometry_msgs::msg::PoseStamped pose;

  util_->updateTaskId(task_id);
  util_->setActiveTaskPtr(autonomous_driving_);

  std::thread t([&]{ autonomous_driving_->execute(task_id, pose); });
  // allow retries + final timeout
  std::this_thread::sleep_for(std::chrono::duration<double>((MAX_DRIVE_RETRIES + 1) * DRIVE_WAIT_TIMEOUT_S + 0.5));
  if (t.joinable()) t.join();

  EXPECT_EQ(util_->getTaskStatus(task_id).status, "TIMEOUT");
}

// Test 3: Successful Execution (drive to arrived)
TEST_F(AutonomousDrivingTest, TestAutonomousDrivingSuccess) {
  std::string task_id = "autonomous_drive_success";
  geometry_msgs::msg::PoseStamped pose;

  util_->updateTaskId(task_id);
  util_->setActiveTaskPtr(autonomous_driving_);
  autonomous_driving_->forceSetState(AutonomousDrivingTaskState::DRIVING);

  autoware_adapi_v1_msgs::msg::RouteState rs;
  rs.state = autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
  autonomous_driving_->forceSetRouteState(rs);

  std::thread t([&]{ autonomous_driving_->execute(task_id, pose); });
  spinFor(0.5);
  t.join();

  EXPECT_EQ(util_->getTaskStatus(task_id).status, "SUCCESS");
}

// Test 4: Retry-Once Branch -> status should go to RETRYING
TEST_F(AutonomousDrivingTest, TestAutonomousDrivingRetryOnce) {
  std::string task_id = "autonomous_drive_retry";
  geometry_msgs::msg::PoseStamped pose;

  util_->updateTaskId(task_id);
  util_->setActiveTaskPtr(autonomous_driving_);

  // ensure initial WAIT fails by not setting autonomous mode
  autoware_adapi_v1_msgs::msg::OperationModeState om;
  om.mode = autoware_adapi_v1_msgs::msg::OperationModeState::STOP;
  autonomous_driving_->forceSetOperationModeState(om);

  std::thread t([&]{ autonomous_driving_->execute(task_id, pose); });

  // wait until RETRYING appears
  bool saw_retry = false;
  for (int i = 0; i < 50; ++i) {
    if (util_->getTaskStatus(task_id).status == "RETRYING") { saw_retry = true; break; }
    std::this_thread::sleep_for(100ms);
    autonomous_driving_->simulateDriveWaitTimeout();
  }
  EXPECT_TRUE(saw_retry) << "Did not see RETRYING status";

  // cancel to exit
  autonomous_driving_->cancel();
  t.join();
}

// Test 5: HALTED Condition in DRIVING Branch
TEST_F(AutonomousDrivingTest, TestAutonomousDrivingHalted) {
  std::string task_id = "autonomous_drive_halted";
  geometry_msgs::msg::PoseStamped pose;

  util_->updateTaskId(task_id);
  util_->setActiveTaskPtr(autonomous_driving_);

  // set DRIVING and STOPPED
  autonomous_driving_->forceSetState(AutonomousDrivingTaskState::DRIVING);
  autoware_adapi_v1_msgs::msg::MotionState ms;
  ms.state = autoware_adapi_v1_msgs::msg::MotionState::STOPPED;
  autonomous_driving_->forceSetVehicleMotionState(ms);

  // age the halt timer
  autonomous_driving_->simulateHaltTimeout();

  std::thread t([&]{ autonomous_driving_->execute(task_id, pose); });

  // poll for HALTED
  bool saw_halt = false;
  for (int i = 0; i < 50; ++i) {
    if (util_->getTaskStatus(task_id).status == "HALTED") { saw_halt = true; break; }
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_TRUE(saw_halt) << "Did not see HALTED status";

  // unblock and finish
  autoware_adapi_v1_msgs::msg::RouteState rs;
  rs.state = autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
  autonomous_driving_->forceSetRouteState(rs);
  ms.state = autoware_adapi_v1_msgs::msg::MotionState::MOVING;
  autonomous_driving_->forceSetVehicleMotionState(ms);

  t.join();
  EXPECT_EQ(util_->getTaskStatus(task_id).status, "SUCCESS");
}

// Test 6: DRIVING Branch Recovery to Success
TEST_F(AutonomousDrivingTest, TestAutonomousDrivingDrivingToSuccess) {
  std::string task_id = "autonomous_drive_move_success";
  geometry_msgs::msg::PoseStamped pose;

  util_->updateTaskId(task_id);
  util_->setActiveTaskPtr(autonomous_driving_);
  autonomous_driving_->forceSetState(AutonomousDrivingTaskState::DRIVING);

  autoware_adapi_v1_msgs::msg::MotionState ms;
  ms.state = autoware_adapi_v1_msgs::msg::MotionState::MOVING;
  autonomous_driving_->forceSetVehicleMotionState(ms);

  autoware_adapi_v1_msgs::msg::RouteState rs;
  rs.state = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
  autonomous_driving_->forceSetRouteState(rs);

  std::thread t([&]{ autonomous_driving_->execute(task_id, pose); });
  std::this_thread::sleep_for(200ms);

  rs.state = autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
  autonomous_driving_->forceSetRouteState(rs);

  t.join();
  EXPECT_EQ(util_->getTaskStatus(task_id).status, "SUCCESS");
}

// -----------------------------------------------------------------------------
// 7. engageAutoDrive(): service AVAILABLE branch
// -----------------------------------------------------------------------------
TEST_F(AutonomousDrivingTest, TestEngageAutoDriveServiceAvailable) {
  // we already have a fake_autonomous_svc_ in SetUp, so wait_for_service() returns true immediately
  EXPECT_NO_THROW( autonomous_driving_->engageAutoDrive() );
}

// Test 8: engageAutoDrive(): simulate “never available” and ensure it
// cleanly bails out via the shutdown branch.
TEST_F(AutonomousDrivingTest, TestEngageAutoDriveServiceUnavailable) {
  // 1) Remove the initial fake so wait_for_service(1s) times out once:
  fake_autonomous_svc_.reset();

  // 2) After ~1.1 s, recreate the service with the proper callback signature:
  std::thread bringup([&]() {
    std::this_thread::sleep_for(1100ms);
    fake_autonomous_svc_ = node->create_service<
      autoware_adapi_v1_msgs::srv::ChangeOperationMode
    >(
      "/api/operation_mode/change_to_autonomous",
      // NOTE: explicit parameter types
      [](
        const std::shared_ptr<rmw_request_id_t> /*req_header*/,
        const std::shared_ptr<
          autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request
        > /*req*/,
        std::shared_ptr<
          autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response
        > /*res*/
      ) {
        // no-op
      }
    );
  });

  // 3) Call engageAutoDrive(): it will log “service not available” once,
  //    then exit cleanly when the service appears.
  EXPECT_NO_THROW( autonomous_driving_->engageAutoDrive() );

  bringup.join();
}

// -----------------------------------------------------------------------------
// 9. operationModeStateCallback()
// -----------------------------------------------------------------------------
TEST_F(AutonomousDrivingTest, TestOperationModeStateCallback) {
  autoware_adapi_v1_msgs::msg::OperationModeState msg;
  msg.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
  autonomous_driving_->operationModeStateCallback(msg);
  EXPECT_EQ( autonomous_driving_->operation_mode_state_.mode, msg.mode );
}

// -----------------------------------------------------------------------------
// 10. vehicleMotionStateCallback()
// -----------------------------------------------------------------------------
TEST_F(AutonomousDrivingTest, TestVehicleMotionStateCallback) {
  autoware_adapi_v1_msgs::msg::MotionState msg;
  msg.state = autoware_adapi_v1_msgs::msg::MotionState::MOVING;
  autonomous_driving_->vehicleMotionStateCallback(msg);
  EXPECT_EQ( autonomous_driving_->vehicle_motion_state_, msg.state );
}

// -----------------------------------------------------------------------------
// 11. routeStateCallback()
// -----------------------------------------------------------------------------
TEST_F(AutonomousDrivingTest, TestRouteStateCallback) {
  autoware_adapi_v1_msgs::msg::RouteState msg;
  msg.state = autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
  autonomous_driving_->routeStateCallback(msg);
  EXPECT_EQ( autonomous_driving_->route_state_, msg.state );
}

// -----------------------------------------------------------------------------
// 12. cancelCurrentRoute(): service AVAILABLE branch
// -----------------------------------------------------------------------------
TEST_F(AutonomousDrivingTest, TestCancelCurrentRouteServiceAvailable) {
  // fake_clear_route_svc_ is up in SetUp, so wait_for_service() returns true
  EXPECT_NO_THROW( autonomous_driving_->cancelCurrentRoute() );
}

// Test 13: cancelCurrentRoute(): same pattern for the clear_route client.
TEST_F(AutonomousDrivingTest, TestCancelCurrentRouteServiceUnavailable) {
  fake_clear_route_svc_.reset();

  std::thread bringup([&]() {
    std::this_thread::sleep_for(1100ms);
    fake_clear_route_svc_ = node->create_service<
      autoware_adapi_v1_msgs::srv::ClearRoute
    >(
      "/api/routing/clear_route",
      [](
        const std::shared_ptr<rmw_request_id_t> /*req_header*/,
        const std::shared_ptr<
          autoware_adapi_v1_msgs::srv::ClearRoute::Request
        > /*req*/,
        std::shared_ptr<
          autoware_adapi_v1_msgs::srv::ClearRoute::Response
        > /*res*/
      ) {
        // no-op
      }
    );
  });

  EXPECT_NO_THROW( autonomous_driving_->cancelCurrentRoute() );

  bringup.join();
}