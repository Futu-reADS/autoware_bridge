#ifndef BASE_TASK_HPP
#define BASE_TASK_HPP

#include <variant>
#include <string>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"

namespace autoware_bridge
{

// Variant-based task input wrapper
struct TaskInput
{
  std::variant<
    std::monostate,
    geometry_msgs::msg::PoseStamped,
    geometry_msgs::msg::PoseWithCovarianceStamped,
    std_msgs::msg::String
  > data;
};

class BaseTask
{
public:
  virtual ~BaseTask() = default;

  /**
   * @brief Execute the task with the given ID and input.
   * The derived class must handle the appropriate input type via std::get_if or std::get.
   *
   * @param task_id ID of the task, used for logging and tracking.
   * @param input   Input payload containing pose, string, or other task-specific data.
   */
  virtual void execute(
    const std::string & task_id, const TaskInput & input) = 0;

  /**
   * @brief Pure virtual function to request cancellation.
   */
  virtual void cancel() = 0;
};

}  // namespace autoware_bridge

#endif  // BASE_TASK_HPP
