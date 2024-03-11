#include "reference_path.h"

namespace pnc{
namespace control{

ReferencePath::ReferencePath()
{
    // 生成参考轨迹x,y
    std::vector<std::vector<double>> refer_path(1000,std::vector<double>(4));
    for (int i = 0; i < 1000; i++)
    {
        refer_path[i][0] = 0.1 * i;
        refer_path[i][0] = 2 * sin(refer_path[i][0] / 3.0) + 2.5 * cos(refer_path[i][0] / 2.0);
        refer_path.push_back(refer_path[i][0]);
        refer_path.push_back(refer_path[i][0]);
    }
    // 使用差分计算路径点的一阶导和二阶导，从而得到切线方向和曲率
    double dx, dy, ddx, ddy;
    for (int i = 0; i < refer_path.size(); i++)
    {
        if (i == 0)
        {
            dx = refer_path[1][0] - refer_path[0][0];
            dy = refer_path[1][1] - refer_path[0][1];
            ddx = refer_path[2][0] - 2 * refer_path[1][0] + refer_path[0][0];
            ddy = refer_path[2][1] - 2 * refer_path[1][1] + refer_path[0][1];
        }else if (i == refer_path.size() -1)
        {
            dx = refer_path[i][0] - refer_path[i-1][0];
            dy = refer_path[i][1] - refer_path[i-1][1];
            ddx = refer_path[i][0] - 2* refer_path[i-1][0] + refer_path[i-2][0];
            ddy = refer_path[i][1] - 2* refer_path[i-1][1] + refer_path[i-2][1];
        }else
        {
            dx = refer_path[i][0] - refer_path[i-1][0];
            dy = refer_path[i][1] - refer_path[i-1][1];
            ddx = refer_path[i+1][0] - 2* refer_path[i][0] + refer_path[i-1][0];
            ddy = refer_path[i+1][1] - 2* refer_path[i][1] + refer_path[i-1][1];
        }
        refer_path[i][2] = atan2(dy, dx);  // heading
        // 计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
        // 参考：https://blog.csdn.net/weixin_46627433/article/details/123403726
        refer_path[i][3] = (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 3/2); //  kappa
    }

}

std::vector<double> ReferencePath::calcTrackError(const std::vector<double> vehicle_state)
{
    double x = vehicle_state[0];
    double y = vehicle_state[1];
    std::vector<double> d_x(refer_path.size());
    std::vector<double> d_y(refer_path.size());
    std::vector<double> dists(refer_path.size());

    for (int i = 0; i < refer_path.size(); i++)
    {
        d_x[i] = refer_path[i][0] - x;
        d_y[i] = refer_path[i][1] - y;
        dists[i] = sqrt(d_x[i] * d_x[i] + d_y[i] * d_y[i]);
    }
    int min_index = std::min_element(dists.begin(), dists.end()) - dists.end();
    double yaw = refer_path[min_index][2];
    double kappa = refer_path[min_index][3];
    double error = dists[min_index];

    double angle = normalizeAngle(yaw - atan2(d_y[min_index], d_x[min_index]));
    if (angle < 0)
    {
        error *= -1;
    }
    return {error, kappa, yaw, min_index};
}

double ReferencePath::normalizeAngle(double angle)
{
    if (angle > 2*PI)
    {
        angle -= 2*PI;
    }else if (angle < -2*PI)
    {
        angle += 2*PI;
    }   
    return angle;
}

refTraj ReferencePath::calcRefTrajectory(std::vector<double> vehcile_state, parameters param, double dl)
{
    std::vector<double> result = calcTrackError(vehcile_state);
    double err = result[0], kappa = result[1], yaw = result[2], min_index = result[3];

    refTraj ref_traj;
    ref_traj.xref = Matrix(param.NX, param.Hp + 1);
    ref_traj.dref = Matrix(param.NU, param.Hp);

    // 参考控制量
    double ref_delta = atan2(param.L * kappa, 1);
    for (int i = 0; i < param.Hp; i++)
    {
        ref_traj.dref(0, i) = vehcile_state[3];
        ref_traj.dref(1, i) = ref_delta;
    }
    
    // 参考点的初始点
    ref_traj.xref(0, 0) = refer_path[min_index][0];
    ref_traj.xref(1, 0) = refer_path[min_index][1];
    ref_traj.xref(2, 0) = refer_path[min_index][2];

    int len = refer_path.size();
    double s = 0;
    for (int i = 0; i < param.Hp + 1; i++)
    {
        s += abs(vehcile_state[3]) * param.dt;
        int dind = (int)round(s / dl);
        if (min_index + dind < len)
        {
            ref_traj.xref(0, i) = refer_path[index + dind][0];
            ref_traj.xref(1, i) = refer_path[index + dind][1];
            ref_traj.xref(2, i) = refer_path[index + dind][2];
        }else
        {
            ref_traj.xref(0, i) = refer_path[len - 1][0];
            ref_traj.xref(1, i) = refer_path[len - 1][1];
            ref_traj.xref(2, i) = refer_path[len - 1][2];
        }
    }
    return refTraj();
}
}  // namespace control
}  // namespace pnc