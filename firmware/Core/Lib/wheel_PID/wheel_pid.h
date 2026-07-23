#pragma once

class PID{
    public:
    PID(double kp,double ki,double kd,double output_max = 100,double output_min = -100);
    void setGains(double kp,double ki,double kd);
    void setOutputBoundaries(double output_max,double output_min);
    void setPoint(double target);
    double update(double measured,double dt);
    void reset();

    double kp,ki,kd;
    double output_max;
    double output_min;
    double integral_max = 100;
    double integral_min = 100;

    double integral = 0;
    double prev_measurement = 0;
    double setpoint;
    double output;
    private:
};