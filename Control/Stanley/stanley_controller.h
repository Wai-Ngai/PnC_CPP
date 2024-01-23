#pragma once

#include <vector>
#define PI 3.1415926

namespace pnc{
namespace control{

class StanleyController
{
public:
    StanleyController() = default;

    /**
     * @brief 计算距离自车最近点的index
     * 
     * @param vehcile_state 
     * @param reference_path 
     * @return double 
     */
    double CalcTargetIndex(const std::vector<double> vehcile_state, const std::vector<std::vector<double>> reference_path);

    /**
     * @brief 角度归一化至[-pi pi]
     * 
     * @param angle 
     * @return double 
     */
    double NormalizeAngle(double angle);

    /**
     * @brief 计算前轮转角，Stanley算法
     * 
     * @param vehcile_state 
     * @param reference_path 
     * @return std::vector<double> 
     */
    std::vector<double> Control(const std::vector<double> vehcile_state, const std::vector<std::vector<double>> reference_path);

    ~StanleyController() = default;
private:
    double k_{3};
};
}   // namespace control
}   // namespace pnc