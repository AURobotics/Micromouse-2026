#pragma once
#include "feedforward_pi.h"

VelocityController::VelocityController() = default;

explicit VelocityController::VelocityController(const FFPIConfig& config)
    : cfg_(config) {}

void VelocityController::setConfig(const FFPIConfig& config) {
    cfg_ = config;
}

const FFPIConfig& VelocityController::getConfig() const {
    return cfg_;
}

void VelocityController::reset() {
    integral_ = 0.0f;
    lastDebug_ = FFPIDebug();
}

//main control loop returns pwm value
float VelocityController::compute(float targetVelocity, float targetAcceleration,float actualVelocity, float dt) {
    if (dt <= 0.0f) {
        return lastDebug_.output;
    }
        
    //feedforward
    float sign = 0.0f;
    if (targetVelocity > 0.0f) sign = 1.0f;
    else if (targetVelocity < 0.0f) sign = -1.0f;

    float ff = cfg_.kS*sign + cfg_.kV*targetVelocity + cfg_.kA*targetAcceleration;

    //PI
    float error = targetVelocity - actualVelocity;
    float pTerm = cfg_.kP * error;

    float integral = integral_ + error * dt;
    integral = clamp(integral, cfg_.integralMin, cfg_.integralMax);
    float iTerm = cfg_.kI * integral;

    float rawOutput = ff + pTerm + iTerm;
    float output = clamp(rawOutput, cfg_.outputMin, cfg_.outputMax);

    bool saturated = rawOutput != output;
    bool wouldReduceSaturation =
    (rawOutput > cfg_.outputMax && error < 0.0f) ||
    (rawOutput < cfg_.outputMin && error > 0.0f);
    
    //only accumulate if not saturated or if accumulating would reduce saturation
    if (!saturated || wouldReduceSaturation) {
        integral_ = integral;
    }

    lastDebug_.targetVelocity = targetVelocity;
    lastDebug_.targetAcceleration = targetAcceleration;
    lastDebug_.actualVelocity = actualVelocity;
    lastDebug_.error = error;
    lastDebug_.ffTerm = ff;
    lastDebug_.pTerm = pTerm;
    lastDebug_.iTerm = iTerm;
    lastDebug_.output = output;
    lastDebug_.saturated = saturated;

    return output;
}

//for logging
const FFPIDebug& VelocityController::getDebug() const {
    return lastDebug_;
}

float VelocityController::getIntegral() const {
    return integral_;
}


float VelocityController::clamp(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}
