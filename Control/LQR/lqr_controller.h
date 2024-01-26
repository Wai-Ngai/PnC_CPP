#pragma once

#include <Eigen/Dense>
#include <vector>

#define EPS 1.0e-4


namespace pnc{
namespace control{

using Matrix = Eigen::MatrixXd;

class LQRController
{
public:
    /**
     * @brief Construct a new LQRController object
     * 
     * @param max_iteration 黎卡提方程最大迭代次数
     */
    LQRController(int max_iteration);

    /**
     * @brief 解代数黎卡提方程
     * 
     * @param A 状态矩阵A
     * @param B 状态矩阵B
     * @param Q Q为半正定的状态加权矩阵, 通常取为对角阵；Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零；
     * @param R R为正定的控制加权矩阵，R矩阵元素变大意味着希望控制输入能够尽可能小。
     * @return Matrix 
     */
    Matrix CalcRicatti(const Matrix A, const Matrix B, const Matrix Q, const Matrix R);

    /**
     * @brief 
     * 
     * @param vehcile_state 
     * @param refer_path 
     * @param s0 
     * @param A 
     * @param B 
     * @param Q 
     * @param R 
     * @return double 
     */
    double Control(const std::vector<double> vehcile_state, const std::vector<std::vector<double>> refer_path,
                   const double min_index, const Matrix A, const Matrix B, const Matrix Q, const Matrix R);

    ~LQRController() = default;

private:
    int max_num_iteration_;
};
}   // namespace control
}   // namespace pnc
