// BaseTask.hpp
#ifndef BASE_TASK_HPP
#define BASE_TASK_HPP

#include <string>

class BaseTask
{
public:
  virtual ~BaseTask() = default;

  // Pure virtual function to execute the task.
  virtual void execute(const std::string & task_id) = 0;

  // Pure virtual function to request cancellation.
  virtual void request_cancel() = 0;
};

#endif  // BASE_TASK_HPP
