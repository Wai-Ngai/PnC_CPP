#ifndef PNC_CPP_KINEMATICMODEL_H
#define PNC_CPP_KINEMATICMODEL_H
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class KinematicModel
{
public:
    double x, y, psi, v, L, dt;

public:
    KinematicModel();

    /**
     * 机器人运动学模型构造
     * @param x 位置x
     * @param y 位置y
     * @param psi 偏航角
     * @param v 速度
     * @param l 轴距
     * @param dt 采样时间
     */
    KinematicModel(double x, double y, double psi, double v, double l, double dt);

    /**
     * 状态获取
     * @return
     */
    vector<double> getState();

    /**
     * 控制量为转向角delta_f和加速度a
     * @param accel 加速度
     * @param delta_f 转向角控制量
     */
    void updateState(double accel, double delta_f);
    
    /**
     * 将模型离散化后的状态空间表达
     * @param ref_delta 名义控制输入
     * @param ref_yaw 名义偏航角
     * @return
     */
    vector<MatrixXd> stateSpace(double ref_delta, double ref_yaw);
};

#endif // PNC_CPP_KINEMATICMODEL_H
