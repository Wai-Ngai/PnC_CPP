#include "kinematic_model.h"

namespace pnc {
namespace control {

KinematicModel::KinematicModel(double x, double y, double psi, double v,
                               double l, double dt)
    : x_(x), y_(y), psi_(psi), v_(v), l_(l), dt_(dt) {}

void KinematicModel::updateState(double accel, double delta_f) {
  x_ += v_ * cos(psi_) * dt_;
  y_ += v_ * sin(psi_) * dt_;
  psi_ += v_ / l_ * tan(delta_f) * dt_;
  v_ += accel * dt_;
}

std::vector<double> KinematicModel::getState() const {
  return {x_, y_, psi_, v_};  // c++11 中的列表初始化语法
}

std::vector<Matrix> KinematicModel::stateSpace(double delta_ref,
                                               double yaw_ref) {
  Matrix A = Matrix::Zero(3, 3);
  Matrix B = Matrix::Zero(3, 2);

  /*
      A_matrix
      [1 0 -v*sin(yaw)*dt;
       0 1  v*cos(yaw)*dt;
       0 0  1;]

      B_matrix
      [cos(yaw)*dt      0;
       sin(yaw)*dt      0;
       tan(delta)/l*dt  v*dt/(l*cos(delta)*cos(delta))]
  */

  A << 1.0, 0.0, -v_ * sin(yaw_ref) * dt_, 0.0, 1.0, v_ * cos(yaw_ref) * dt_,
      0.0, 0.0, 1.0;
  B << cos(yaw_ref) * dt_, 0.0, sin(yaw_ref) * dt_, 0.0,
      tan(delta_ref) / l_ * dt_,
      v_ * dt_ / (l_ * cos(delta_ref) * cos(delta_ref));
  return {A, B};
}

}  // namespace control
}  // namespace pnc
