/**
 * @file 车辆运动学模型实现
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

# pragma once

#include <vector>
#include <Eigen/Dense>

namespace pnc{
namespace control{

using Matrix = Eigen::MatrixXd;

class KinematicModel {
public:
    KinematicModel() = default;

    /**
     * @brief 有参构造函数
     * 
     * @param x     x向坐标
     * @param y     y向坐标
     * @param psi   横摆角
     * @param v     速度
     * @param l     轴距
     * @param dt    采样时间
     */
    KinematicModel(double x, double y, double psi, double v, double l, double dt);

    /**
     * @brief 获取最新的状态
     * 
     * @return std::vector<double> 
     */
    std::vector<double> getState() const;

    /**
     * @brief 根据转角和加速度，更新车辆状态
     * 
     * @param accel   加速度
     * @param delta   前轮转角
     */
    void updateState(double accel, double delta);
    
    /**
     * @brief 将模型离散化后的状态空间矩阵A,B
     * 
     * @param delta_ref   参考控制输入
     * @param yaw_ref     参考横摆角
     * @return std::vector<Matrix> 
     */
    std::vector<Matrix> stateSpace(double delta_ref, double yaw_ref);

    ~KinematicModel() = default;
public:
    double x_;
    double y_;
    double psi_;
    double v_;
    double l_;
    double dt_;
};

}  // namespace control
}  // namespace pnc
