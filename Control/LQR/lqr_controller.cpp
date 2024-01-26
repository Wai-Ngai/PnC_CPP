#include "lqr_controller.h"

namespace pnc{
namespace control{

LQRController::LQRController(int max_iteration) : max_num_iteration_(max_iteration){}

Matrix LQRController::CalcRicatti(const Matrix A, const Matrix B, const Matrix Q, const Matrix R)
{
    Matrix P = Q;
    Matrix AT = A.transpose();
    Matrix BT = B.transpose();
    
    uint num_iteration = 0;
    double diff = std::numeric_limits<double>::max();

    while (num_iteration++ < max_num_iteration_ && diff > EPS)
    {
        Matrix P_next = Q + AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A;
        diff = fabs((P - P_next).maxCoeff());
        P = P_next;
    }

    Matrix K = (R + BT * P * B).inverse() * BT * P * A;
    return K;
}

double LQRController::Control(const std::vector<double> vehcile_state, const std::vector<std::vector<double>> refer_path,
                              const double min_index, const Matrix A, const Matrix B, const Matrix Q, const Matrix R)
{
    Matrix X(3,1);
    X << vehcile_state[0] - refer_path[min_index][0],
         vehcile_state[1] - refer_path[min_index][1],
         vehcile_state[2] - refer_path[min_index][2];
    
    Matrix K = CalcRicatti(A, B, Q, R);
    Matrix u = -K * X;

    return u(1,0);
}


}   // namespace control
}   // namespace pnc
