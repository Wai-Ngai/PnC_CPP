#include "stanley_controller.h"
#include "../utils/KinematicModel.h"
#include "../../matplotlibcpp.h"

namespace plt = matplotlibcpp;
using  pnc::control::KinematicModel;
using  pnc::control::StanleyController;


int main(int argc, const char** argv) {
    double x0 = 0.0, y0 = -1.0, psi = 0.5, v = 2, L = 2, dt = 0.1;
    // 预瞄距离参数
    double kd = 3.0, c0 = 2;

    // 生成轨迹
    std::vector<std::vector<double>> reference_path(1000,std::vector<double>(3));
    std::vector<double> reference_path_x, reference_path_y;
    for(int i = 0; i < 1000; i++)
    {
        reference_path[i][0] = 0.1 * i;
        reference_path[i][1] = 2 * sin(reference_path[i][0] / 3.0) + 2.5 * cos(reference_path[i][0] / 2.0);

        reference_path_x.push_back(reference_path[i][0]);
        reference_path_y.push_back(reference_path[i][1]);
    }
    for (int i = 0; i < reference_path.size() - 1; i++) 
    {
        // 计算前后两个点之间的 x、y 坐标差值
        double dx = reference_path[i + 1][0] - reference_path[i][0];
        double dy = reference_path[i + 1][1] - reference_path[i][1];
        
        // 计算航向角（弧度）
        double heading = atan2(dy, dx);
        
        reference_path[i][2] = heading;
    }

    std::cout << "生成的轨迹：" << std::endl;
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
    std::vector<double> vehcile_state(4);

    StanleyController stanley;
    for (int i = 0; i < 600; i++)
    {
        plt::clf();
        
        vehcile_state = model.getState();
        double ld = kd * model.v_ + c0;
        auto result = stanley.Control(vehcile_state, reference_path);

        model.updateState(0, result[0]);

        vehcile_x.push_back(model.x_);
        vehcile_y.push_back(model.y_);
        
        // plot
        plt::plot(reference_path_x, reference_path_y, "b--");
        plt::plot(vehcile_x, vehcile_y, "r");
        plt::grid(true);
        plt::ylim(-5, 5);
        plt::pause(0.01);

        if (result[1] >= reference_path.size() - 1)
        {
            break;
        }
        
    }
    // save figure
    const char* filename = "./stanley_controller.png";
    std::cout << "saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}