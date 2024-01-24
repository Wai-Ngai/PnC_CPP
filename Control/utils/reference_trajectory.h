#pragma once

#include <vector>

namespace pnc{
namespace control{

struct PathPoint
{
    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> path_heading;
    std::vector<double> path_kappa;
};


class ReferenceTrajectory
{
public:
    /**
     * @brief 构造函数，初始化参考轨迹信息
     * 
     */
    ReferenceTrajectory();

    double NormalizeAngle(double angle);

    std::vector<double> CalcTrackError(std::vector<double> vehcile_state);

    ~ReferenceTrajectory() = default;

public:
    PathPoint path_point_;
};

}   // namespace control
}   // namespace pnc