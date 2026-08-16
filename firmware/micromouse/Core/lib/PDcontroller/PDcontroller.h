#pragma once

class PDController {
public:
    PDController(float kp, float kd, float outputMin, float outputMax){
        kp_ = kp;
        kd_ = kd;
        outputMin_ = outputMin;
        outputMax_ = outputMax;
        reset();
    }
    void reset(){
        prevError_ = 0.0f;
        firstCall_ = true;
    }
    float compute(float setpoint, float measured, float dt){
        float error = setpoint - measured;

        float derivative = 0.0f;
        if (!firstCall_ && dt > 0.0f) {
            derivative = (error - prevError_) / dt;
        }
        firstCall_ = false;
        prevError_ = error;

        float output = kp_ * error + kd_ * derivative;

        if (output > outputMax_) output = outputMax_;
        if (output < outputMin_) output = outputMin_;

        return output;
    }

private:
    float kp_ = 0.0f;
    float kd_ = 0.0f;
    float outputMin_ = -1.0f;
    float outputMax_ = 1.0f;
    float prevError_ = 0.0f;
    bool  firstCall_ = true;
};