#pragma once
#include <vector>
#include <Eigen/Dense>

#define PI 3.1415926

namespace pnc{
namespace control{

using Matrix = Eigen::MatrixXd;

struct refTraj
{
    Matrix xref,dref;
    int index;
};

struct parameters
{
    int L;
    int NX,XU,Hp;
    double dt;
};


class ReferencePath
{
public:
    /**
     * @brief Construct a new Reference Path
     * 
     */
    ReferencePath();

    /**
     * @brief 计算跟踪误差
     * 
     * @param vehicle_state 整车状态 
     * @return std::vector<double> 
     */
    std::vector<double> calcTrackError(const std::vector<double> vehicle_state);

    /**
     * @brief 角度归一化
     * 
     * @param angle 
     * @return double 
     */
    double normalizeAngle(double angle);

    /**
     * @brief 计算参考轨迹点和参考控制量，用于MPC计算
     * 
     * @param vehcile_state 整车状态
     * @param param 
     * @param dl 
     * @return refTraj 
     */
    refTraj calcRefTrajectory(std::vector<double> vehcile_state, parameters param, double dl = 1.0);

    ~ReferencePath() = default;
public:
    std::vector<std::vector<double>> refer_path; // x,y,heading,kappa
    std::vector<double> refer_x,ref_y;
};
}  // namespace control
}  // namespace pnc