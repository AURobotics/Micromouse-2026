#pragma once
//VelocityController
//Feedforward + PI velocity controller for a single wheel.


struct FFPIConfig {
    //Feedforward
    float km_ff = 0.05f;   // static friction term  (smallest pwm to get the wheels moving)
    float tau_ff = 0.12f;

    //PI
    float kP = 0.0f;
    float kI = 0.0f;

    // Output / integral limits 
    float outputMin = -100.0f;
    float outputMax = 100.0f;
    float integralMin = -100.0f;
    float integralMax = 100.0f;
};

struct FFPIDebug {
    float targetVelocity = 0.0f;
    float actualVelocity = 0.0f;
    float error = 0.0f;
    float ffTerm = 0.0f;
    float pTerm = 0.0f;
    float iTerm = 0.0f;
    float output = 0.0f;
    bool saturated = false;
};

class VelocityController {
public:
    VelocityController() = default;

    explicit VelocityController(const FFPIConfig& config);
    void setConfig(const FFPIConfig& config);
    const FFPIConfig& getConfig() const;
    void reset();
    float compute(float targetVelocity, float actualVelocity, float dt);
    const FFPIDebug& getDebug() const;
    float getIntegral() const;

private:
    static float clamp(float value, float lo, float hi);

    FFPIConfig cfg_;
    FFPIDebug lastDebug_;
    float integral_ = 0.0f;
    float prevTargetVelocity_ = 0.0f;
    bool firstCall_ = true;
};

