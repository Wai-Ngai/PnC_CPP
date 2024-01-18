#include "pid_controller.h"

namespace pnc{
namespace control{

PIDController::PIDController(double kp,double ki, double kd, double upper, double lower)
                            :kp_(kp),ki_(ki),kd_(kd),upper_(upper),lower_(lower){
    previous_error_ = 0;
    integral_ = 0;
    first_hit_ = true;
}

void PIDController::setGain(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setBound(double upper, double lower)
{
    upper_ = upper;
    lower_ = lower;
}

void PIDController::reset()
{
    previous_error_ = 0;
    integral_ = 0;
}

double PIDController::calcOutput(double error, double dt)
{
    double output = 0;
    double diff = 0;
    
    if (first_hit_)
    {
        first_hit_ = false;
    }else{
        diff = (error - previous_error_)/dt;
    }
    
    integral_ += ki_ * error * dt;
    if (output > upper_)
    {
        output = upper_;
    }else if (output < lower_)
    {
        output = lower_;
    }    

    output = kd_ * error + integral_ + kd_ * diff;
    previous_error_ = error;
    return output;
}
}   // namespace control
}   // namespace pnc