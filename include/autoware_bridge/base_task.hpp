// BaseTask.hpp
#ifndef BASE_TASK_HPP
#define BASE_TASK_HPP

#include "autoware_bridge/autoware_bridge_util.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>

class BaseTask
{
public:
  virtual ~BaseTask() = default;

  // The standard execute function.
  virtual void execute(
    const std::string & task_id, const geometry_msgs::msg::PoseStamped & pose) = 0;

  // Pure virtual function to request cancellation.
  virtual void cancelRequested() = 0;
};

#endif  // BASE_TASK_HPP
