#include "kinematic_model.h"

#include <cmath>

namespace pnc{
namespace control{
using Matrix = Eigen::MatrixXd;

KinematicModel::KinematicModel(double x, double y, double psi, double v, double l, double dt)
                              :x_(x),y_(y),psi_(psi),v_(v),l_(l),dt_(dt){}


void KinematicModel::updateState(double accel, double delta_f)
{
    x_ += v_ * cos(psi_) * dt_;
    y_ += v_ * sin(psi_) * dt_;
    psi_ += v_ / l_ * tan(delta_f) * dt_;
    v_ += accel * dt_;
}

std::vector<double> KinematicModel::getState()
{
    return {x_,y_,psi_,v_}; //  C++11 中的列表初始化语法
}

std::vector<Matrix> KinematicModel::stateSpace(double ref_delta, double ref_yaw)
{

    Matrix A = Matrix::Zero(3, 3);
    Matrix B = Matrix::Zero(3, 2);

    /*
        A_matrix
        [1 0 -v*sin(yaw)*dt;
         0 1  v*cos(yaw)*dt;
         0 0  1;]
        B_matrix
        [cos(yaw)*dt   0;
         sin(yaw)*dt   0;
         tan(delta)/l*dt  v*dt/(l*cos(delta)*cos(delta))]
    */
    A(0,0) = 1.0;
    A(0,2) = - v_ * sin(ref_yaw) * dt_;
    A(1,1) = 1.0;
    A(1,2) = v_ * cos(ref_yaw) * dt_;
    A(2,2) = 1.0;

    B(0,0) = cos(ref_yaw) * dt_;
    B(1,0) = sin(ref_yaw) * dt_;
    B(2,0) = tan(ref_delta)/l_ * dt_;
    B(2,1) = v_*dt_/(l_ * cos(ref_delta)*cos(ref_delta));
   
    return {A,B};
}

}  // namespace control
}  // namespace pnc