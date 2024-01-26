#include "lqr_controller.h"
#include "../utils/MyReferencePath.h"
#include "../utils/kinematic_model.h"
#include "../../matplotlibcpp.h"


namespace plt = matplotlibcpp;
using pnc::control::KinematicModel;
using pnc::control::LQRController;


int main(int argc, char const *argv[])
{
    double dt = 0.1;  // 时间间隔，单位：s
    double L = 2;     // 车辆轴距，单位：m
    double v = 2;     // 初始速度
    double x_0 = 0;   // 初始x
    double y_0 = 0;   // 初始y
    double psi_0 = 0; // 初始航向角
    int max_num_iteration = 100;      // 迭代范围

    MatrixXd Q(3, 3);
    Q << 3.0, 0.0, 0.0,
         0.0, 3.0, 0.0,
         0.0, 0.0, 3.0;
    
    MatrixXd R(2, 2);
    R << 2.0, 0.0,
         0.0, 2.0;

    std::vector<double> vehcile_x, vehcile_y;
    std::vector<double> vehcile_state;
    MyReferencePath reference_path;
    KinematicModel model(x_0, y_0, psi_0, v, L, dt);
    LQRController lrq(max_num_iteration);

    for (int i = 0; i < 500; i++)
    {
        plt::clf();

        vehcile_state = model.getState();
        std::vector<double> one_trial = reference_path.calcTrackError(vehcile_state);
        double kappa = one_trial[1];
        double ref_yaw = one_trial[2];
        double min_index = one_trial[3];

        double ref_delta = atan2(L * kappa, 1);
        std::vector<MatrixXd> state_space = model.stateSpace(ref_delta, ref_yaw);

        double delta_f = lrq.Control(vehcile_state, reference_path.refer_path, min_index, state_space[0], state_space[1], Q, R);
        delta_f += ref_delta;

        model.updateState(0, delta_f);

        vehcile_x.push_back(model.x_);
        vehcile_y.push_back(model.y_);

        // plot
        plt::plot(reference_path.refer_x, reference_path.refer_y, "b--");
        plt::plot({reference_path.refer_x[min_index]}, {reference_path.refer_y[min_index]}, "go");
        plt::plot(vehcile_x, vehcile_y, "r");
        plt::grid(true);
        plt::ylim(-5, 5);
        plt::pause(0.01);
    }
    // save
    const char* filename = "./lqr_demo_png";
    std::cout << " Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();

    return 0;
}
