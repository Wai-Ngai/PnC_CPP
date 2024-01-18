#include "pure_pursuit.h"

#include <cmath>
#include <algorithm>

namespace pnc{
namespace control{

double PurePursuit::CalcTargetIndex(std::vector<double> vehcile_state, std::vector<std::vector<double>> reference_path, double ld)
{
    // 找到距离自车最近点
    std::vector<double> distance;
    for (const auto xy : reference_path)
    {
        double dist = sqrt(pow(xy[0] - vehcile_state[0], 2) + pow(xy[1] - vehcile_state[1], 2));
        distance.push_back(dist);
    }
    double min_index = std::min_element(distance.begin(), distance.end()) - distance.begin();
    
    // 找到距离自车距离大于预瞄距离ld的点的index
    double delta_l = sqrt(pow(reference_path[min_index][0] - vehcile_state[0], 2) + pow(reference_path[min_index][1] - vehcile_state[1], 2));
    while (ld > delta_l && min_index < reference_path.size() -1)
    {
        min_index ++;
        delta_l = sqrt(pow(reference_path[min_index][0] - vehcile_state[0], 2) + pow(reference_path[min_index][1] - vehcile_state[1], 2));
    }
    return min_index;
}

double PurePursuit::Control(std::vector<double> vehcile_state, std::vector<double> reference_point, double ld, double psi, double L)
{
    double alpha = atan2(reference_point[1] - vehcile_state[1], reference_point[0] - vehcile_state[0]) - psi;
    double delta_f = atan2(2 * L * sin(alpha), ld);
    return delta_f;
}

}   // namespace pnc
}   // namespace control