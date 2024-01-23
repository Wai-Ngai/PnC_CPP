#include "stanley_controller.h"
#include <algorithm>
#include <cmath>

namespace pnc{
namespace control{

double StanleyController::CalcTargetIndex(const std::vector<double> vehcile_state, 
                                          const std::vector<std::vector<double>> reference_path)
{
    std::vector<double> distances;
    for (const auto xy : reference_path)
    {
        double dist = sqrt(pow(xy[0] - vehcile_state[0], 2) + pow(xy[1] - vehcile_state[1], 2));
        distances.push_back(dist);
    }
    return std::min_element(distances.begin(), distances.end()) - distances.begin();
}

double StanleyController::NormalizeAngle(double angle)
{
    if (angle > PI)
    {
        angle -= 2*PI;
    }else if (angle < -PI)
    {
        angle += 2*PI;
    }
    return angle;
}

// 计算需要的误差，包括横向误差，航向误差
void StanleyController::ComputeLateralErrors(const double x, const double y,
                                             const double theta, double &e_y,
                                             double &e_theta)
{
    TrajectoryPoint target_point;
    target_point = QueryNearestPointByPosition(x, y);
    
    double heading = target_point.heading;
    const double dx = target_point.x - x;
    const double dy = target_point.y - y;
    e_y = sqrt(dx * dx + dy * dy);

    double fx = dy * cos(heading) - dx * sin(heading); //横向误差的正负
    if(fx > 0){
        e_y = -abs(e_y);
    }else{
        e_y = abs(e_y);
    }

    e_theta = theta - heading; //航向偏差
    if(e_theta > M_PI){
        e_theta = -20*M_PI + e_theta;
    }else if(e_theta < -M_PI){
        e_theta = 20*M_PI + e_theta;
    }
}

std::vector<double> StanleyController::Control(const std::vector<double> vehcile_state, 
                                               const std::vector<std::vector<double>> reference_path)
{
    // 计算规划轨迹中距离自车最近的点信息
    double target_index = CalcTargetIndex(vehcile_state, reference_path);

    std::vector<double> current_ref_point;
    if (target_index >= reference_path.size())
    {
        target_index -= 1; 
    }
    current_ref_point = reference_path[target_index];

    // 计算横向误差lat_error
    double lat_err = 0;
    double psi_r = current_ref_point[2];

    if ((vehcile_state[0] - current_ref_point[0]) * psi_r - (vehcile_state[1] - current_ref_point[1]) > 0)
    {
        lat_err = sqrt(pow(current_ref_point[0] - vehcile_state[0], 2) + pow(current_ref_point[1] - vehcile_state[1], 2));
    }else{
        lat_err = -sqrt(pow(current_ref_point[0] - vehcile_state[0], 2) + pow(current_ref_point[1] - vehcile_state[1], 2));
    }
    
    // Stanley 算法公式
    double psi = vehcile_state[2];
    double v = vehcile_state[3];
    double theta_e = psi_r - psi;
    double delta_e = atan2(k_ * lat_err, v);
    double delta_f = delta_e + NormalizeAngle(theta_e);

    return {delta_f, target_index};
}

}   // namespace control
}   // namespace pnc