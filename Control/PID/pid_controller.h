#pragma once

namespace pnc{
namespace control{

class PIDController
{
public:
    PIDController() = default;

    PIDController(double kp, double ki, double kd, double upper, double lower);

    void setGain(double kp, double ki, double kd);

    void setBound(double upper, double lower);

    double calcOutput(double error, double dt);

    void reset();

    ~PIDController() = default;
private:
    double kp_;
    double ki_;
    double kd_;
    double upper_;
    double lower_;

    double previous_error_;
    double integral_;
    bool first_hit_;
};
}   // namespace control
}   // namespace pnc