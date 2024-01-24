#include "pid_controller.h"
#include "../utils/kinematic_model.h"
#include "../../matplotlibcpp.h"

#include <iostream>
#include <algorithm>

#define PI 3.1415926

using pnc::control::KinematicModel;
using pnc::control::PIDController;
namespace plt = matplotlibcpp;

/**
 * @brief 计算距离参考路径最近点的index
 *
 * @param vehcile_state    车辆位置信息(x,y)
 * @param reference_path   参考路径
 * @return double
 */
double calTargetIndex(std::vector<double> vehcile_state, std::vector<std::vector<double>> reference_path)
{
    std::vector<double> distance;
    for (auto xy : reference_path)
    {
        double dist = sqrt(pow(xy[0] - vehcile_state[0], 2) +
                           pow(xy[1] - vehcile_state[1], 2));
        distance.push_back(dist);
    }
    return std::min_element(distance.begin(), distance.end()) - distance.begin();
}

int main()
{
    // 生成参考轨迹
    std::vector<std::vector<double>> reference_path(1000, std::vector<double>(2));
    std::vector<double> reference_path_x;
    std::vector<double> reference_path_y;
    for (int i = 0; i < 1000; i++)
    {
        reference_path[i][0] = 0.1 * i;
        reference_path[i][1] = 2 * sin(reference_path[i][0] / 3.0);
        reference_path_x.push_back(reference_path[i][0]);
        reference_path_y.push_back(reference_path[i][1]);
    }

    for(const auto& row : reference_path)
    {
        for (const auto& element : row)
        {
            std::cout << element << "\t";
        }
        std::cout << std::endl;
    }

    // 运动学模型
    KinematicModel model(0, -1, 0.05, 2, 2, 0.1);
    // PID控制器
    PIDController PID(2, 0.01, 30, PI / 6, -PI / 6);

    std::vector<double> vehcile_x;
    std::vector<double> vehcile_y;
    std::vector<double> vehcile_state(2);

    for (int i = 0; i < 500; i++)
    {
        plt::clf();
        vehcile_state[0] = model.x_;
        vehcile_state[1] = model.y_;

        double min_index = calTargetIndex(vehcile_state, reference_path);
        double alpha = atan2(reference_path[min_index][1] - vehcile_state[1],
                             reference_path[min_index][0] - vehcile_state[0]);
        double ld = sqrt(pow(reference_path[min_index][0] - vehcile_state[0], 2) +
                         pow(reference_path[min_index][1] - vehcile_state[1], 2));
        double theta = alpha - model.psi_;
        double error_y = ld * sin(theta);
        double delta_f = PID.calcOutput(error_y, 0.01);

        // 更新车辆状态
        model.updateState(0, delta_f);
        vehcile_x.push_back(model.x_);
        vehcile_y.push_back(model.y_);

        // 画图
        plt::plot(reference_path_x, reference_path_y, "b--");
        plt::plot(vehcile_x, vehcile_y, "r");
        plt::grid(true);
        plt::ylim(-2.5, 2.5);
        plt::pause(0.01);
    }
    // 保存图片
    const char *filename = "./pid_test.png";
    std::cout << "saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();

    return 0;
}