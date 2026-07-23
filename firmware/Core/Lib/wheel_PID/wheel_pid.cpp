#include "wheel_pid.h"

PID::PID(double kp,double ki,double kd,double output_max,double output_min){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this-> output_max = output_max;
    this-> output_min = output_min;
}

void PID::setGains(double kp,double ki,double kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PID::setOutputBoundaries(double output_max,double output_min){
    this->output_max = output_max;
    this->output_min = output_min;
}

void PID::setPoint(double target){
    setpoint = target;
}

double PID::update(double measured, double dt){

    double error = setpoint - measured;
    double p_term = kp * error;
    double d_term = -kd * (measured - prev_measurement) / dt;

    double output = p_term + (ki * integral) + d_term;

    bool saturated = (output >= output_max) || (output <= output_min);

    if (!saturated) {
        integral += error * dt;
        if (integral > integral_max) integral = integral_max;
        if (integral < integral_min) integral = integral_min;
    }

    output = p_term + (ki * integral) + d_term;

    if (output > output_max) output = output_max;
    if (output < output_min) output = output_min;

    prev_measurement = measured;
    return output;
}

void PID::reset(){
    prev_measurement = 0;
    integral = 0;
}
