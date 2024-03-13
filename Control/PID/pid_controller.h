/**
 * @file PID控制算法实现
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

namespace pnc {
namespace control {

class PIDController {
 public:
  PIDController() = default;

  PIDController(double kp, double ki, double kd, double upper, double lower);

  void SetGain(double kp, double ki, double kd);

  void SetBound(double upper, double lower);

  double CalcOutput(double error, double dt);

  void Reset();

  ~PIDController() = default;

 private:
  double kp_;
  double ki_;
  double kd_;
  double upper_;
  double lower_;

  double previous_error_;
  double integral_;
  bool first_hit_;
};
}  // namespace control
}  // namespace pnc