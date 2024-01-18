#include "pure_pursuit.h"
#include "../../matplotlibcpp.h"
#include "../utils/KinematicModel.h"

#include <iostream>

#define PI = 3.1415926

namespace plt = matplotlibcpp;
using pnc::control::KinematicModel;
using pnc::control::PurePursuit;



int main(int argc, const char** argv) {
    double x0 = 0.0, y0 = -1.0, psi = 0.5, v = 2, L = 2, dt = 0.1;
    // 预瞄距离参数
    double kd = 0.1, c0 = 2;

    // 生成轨迹
    std::vector<std::vector<double>> reference_path(1000,std::vector<double>(2));
    std::vector<double> reference_path_x, reference_path_y;
    for(int i = 0; i < 1000; i++)
    {
        reference_path[i][0] = 0.1 * i;
        reference_path[i][1] = 2 * sin(reference_path[i][0] / 3.0) + 2.5 * cos(reference_path[i][0] / 2.0);
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
    KinematicModel model(x0,y0,psi,v,L,dt);

    std::vector<double> vehcile_x, vehcile_y;
    std::vector<double> vehcile_state(2);

    PurePursuit pp;
    for (int i = 0; i < 600; i++)
    {
        plt::clf();
        vehcile_state[0] = model.x_;
        vehcile_state[1] = model.y_;

        double ld = kd * model.v_ + c0;
        double min_index = pp.CalcTargetIndex(vehcile_state, reference_path, ld);
        double delta_f = pp.Control(vehcile_state, reference_path[min_index], ld, model.psi_, L);
        model.updateState(0, delta_f);

        vehcile_x.push_back(model.x_);
        vehcile_y.push_back(model.y_);
        
        // plot
        plt::plot(reference_path_x, reference_path_y, "b--");
        plt::plot(vehcile_x, vehcile_y, "r");
        plt::grid(true);
        plt::ylim(-5, 5);
        plt::pause(0.01);
    }
    // save figure
    const char* filename = "./pure_pursuit.png";
    std::cout << "saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}