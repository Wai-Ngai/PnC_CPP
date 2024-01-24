//
// Created by chh3213 on 2022/11/24.
//

#ifndef CHHROBOTICS_CPP_MYREFERENCEPATH_H
#define CHHROBOTICS_CPP_MYREFERENCEPATH_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#define PI 3.1415926

struct refTraj
{
    MatrixXd xref, dref;
    int ind;
};

struct parameters
{
    int L;
    int NX, NU, T;
    double dt;
};

class MyReferencePath
{
public:
    /**
     * 构造函数，求解出参考轨迹点上的曲率等信息
     */
    MyReferencePath();

    /**
     * 计算跟踪误差
     * @param robot_state  机器人状态
     * @return
     */
    vector<double> calcTrackError(vector<double> robot_state);

    /**
     * 角度归一化
     * @param angle
     * @return
     */
    double normalizeAngle(double angle);
    
    /**
     * 计算参考轨迹点，统一化变量数组，只针对MPC优化使用
     * @param robot_state 车辆的状态(x,y,yaw,v)
     * @param param 超參數
     * @param dl Defaults to 1.0.
     * @return {xref, dref, ind}结构体
     */
    refTraj calc_ref_trajectory(vector<double> robot_state, parameters param, double dl = 1.0);

public:
    // refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k
    vector<vector<double>> refer_path;
    vector<double> refer_x, refer_y;
};

#endif // CHHROBOTICS_CPP_MYREFERENCEPATH_H
