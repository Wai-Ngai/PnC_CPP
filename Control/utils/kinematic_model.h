#ifndef PNC_CPP_KINEMATICMODEL_H
#define PNC_CPP_KINEMATICMODEL_H

#include <vector>
#include <Eigen/Dense>

namespace pnc{
namespace control{

class KinematicModel
{
public:
    KinematicModel() = default;

    /**
     * @brief Construct a new Kinematic Model object
     * 
     * @param x    x向坐标
     * @param y    y向坐标
     * @param psi  横摆角
     * @param v    速度
     * @param l    轴距
     * @param dt   采样时间
     */
    KinematicModel(double x, double y, double psi, double v, double l, double dt);

    /**
     * @brief Get the State object
     * 
     * @return std::vector<double> 
     */
    std::vector<double> getState();

    /**
     * @brief  更新状态方程
     * 
     * @param accel     加速度
     * @param delta_f   前轮转角
     */
    void updateState(double accel, double delta_f);

    /**
     * @brief 将模型离散化后的状态空间表达式
     * 
     * @param ref_delta   参考控制输入
     * @param ref_yaw     参考航向角
     * @return std::vector<Matrix> 
     */
    std::vector<Eigen::MatrixXd> stateSpace(double ref_delta, double ref_yaw);

    ~KinematicModel() = default;

public:
    double x_;
    double y_;
    double psi_;
    double v_;
    double l_;
    double dt_;
};

} // namespace control
} // namescpae pnc



#endif // !PNC_CPP_KINEMATICMODEL_H
