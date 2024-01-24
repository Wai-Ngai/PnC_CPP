# include "reference_trajectory.h"

#include <cmath>
#include <algorithm>

namespace pnc{
namespace control{

ReferenceTrajectory::ReferenceTrajectory()
{
    // 生成参考轨迹
    path_point_.path_x = std::vector<double>(1000,0);
    path_point_.path_y = std::vector<double>(1000,0);
    path_point_.path_heading = std::vector<double>(1000,0);
    path_point_.path_kappa = std::vector<double>(1000,0);

    for (int i = 0; i < 1000; i++)
    {
        path_point_.path_x[i] = 0.1 * i;
        path_point_.path_y[i] = 2 * sin(0.1 * i / 3.0) + 2.5 * cos(0.1 * i / 2.0);
    }
    // 使用差分计算路径点的一阶导和二阶导，从而得到切线的方向和曲率
    double dx, dy, ddx, ddy;
    for (int i = 0; i < path_point_.path_x.size(); i++)
    {
        if (i == 0)
        {
            dx = path_point_.path_x[1] - path_point_.path_x[0];
            dy = path_point_.path_y[1] - path_point_.path_y[0];
            ddx = path_point_.path_x[2] - 2 * path_point_.path_x[1] + path_point_.path_x[0];
            ddy = path_point_.path_y[2] - 2 * path_point_.path_y[1] + path_point_.path_y[0];
        }else if (i = path_point_.path_x.size() -1)
        {
            dx = path_point_.path_x[i] - path_point_.path_x[i - 1];
            dy = path_point_.path_y[i] - path_point_.path_y[i - 1];
            ddx = path_point_.path_x[i] - 2 * path_point_.path_x[i - 1] + path_point_.path_x[i - 2];
            ddx = path_point_.path_y[i] - 2 * path_point_.path_y[i - 1] + path_point_.path_y[i - 2];
        }else
        {
            dx = path_point_.path_x[i + 1] - path_point_.path_x[i];
            dy = path_point_.path_y[i + 1] - path_point_.path_y[i];
            ddx = path_point_.path_x[i + 1] - 2 * path_point_.path_x[i] + path_point_.path_x[i - 1];
            ddy = path_point_.path_y[i + 1] - 2 * path_point_.path_y[i] + path_point_.path_y[i - 1];
        }
        // heading
        path_point_.path_heading[i] = atan2(dy, dx);
        // kappa
        path_point_.path_kappa[i] = (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 3 / 2);
    }
}

double ReferenceTrajectory::NormalizeAngle(double angle)
{
    if (angle > PI)
    {
        angle -= 2 * PI;
    }else if (angle < -PI)
    {
        angle += 2 * PI;
    }
    return angle;
}

std::vector<double> ReferenceTrajectory::CalcTrackError(std::vector<double> vehcile_state)
{
    double x = vehcile_state[0];
    double y = vehcile_state[1];
    double size = path_point_.path_x.size();
    std::vector<double> d_x(size);
    std::vector<double> d_y(size);
    std::vector<double> d(size);

    for (int i = 0; i < size; i++)
    {
        d_x[i] = path_point_.path_x[i] - x;
        d_y[i] = path_point_.path_y[i] - y;
        d[i] = sqrt(d_x[i] * d_x[i] + d_y[i] * d_y[i]);
    }
    double min_index = std::min_element(d.begin(), d.end()) - d.begin();

    double heading = path_point_.path_heading[min_index];
    double kappa = path_point_.path_kappa[min_index];
    double error = d[min_index];
    
    double angle = NormalizeAngle(heading - atan2(d_y[min_index], d_x[min_index]));
    if (angle < 0)
    {
        error *= -1;
    }
    return {error, kappa, heading, min_index}
}

}   // namespace pnc
}   // namespace control