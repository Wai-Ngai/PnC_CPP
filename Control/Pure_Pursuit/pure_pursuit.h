#pragma once

#include <vector>

namespace pnc{
namespace control{

class PurePursuit
{
public:
    PurePursuit() = default;

    /**
     * @brief 纯跟踪算法，计算前轮转角
     * 
     * @param vehcile_state      车辆位置
     * @param reference_point    参考轨迹
     * @param ld                 预瞄距离
     * @param psi                横摆角
     * @param L                  轴距
     * @return 前轮转角
     */
    double Control(std::vector<double> vehcile_state, std::vector<double> reference_point, double ld, double psi, double L);

    /**
     * @brief 计算预瞄点index
     * 
     * @param vehcile_state    车辆位置
     * @param reference_path   参考轨迹
     * @param ld               预瞄距离
     * @return 预瞄点index
     */
    double CalcTargetIndex(std::vector<double> vehcile_state, std::vector<std::vector<double>> reference_path, double ld);

    ~PurePursuit() = default;
    
private:
    /* data */
};
}   // namespace pnc
}   // namespace control