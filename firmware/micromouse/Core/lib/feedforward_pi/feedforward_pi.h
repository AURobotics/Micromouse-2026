#pragma once
//VelocityController
//Feedforward + PI velocity controller for a single wheel.


struct FFPIConfig {
    //Feedforward
    float kS = 0.0f;   // static friction term  (smallest pwm to get the wheels moving)
    float kV = 0.0f;   // velocity gain (pwm per unit velocity)
    float kA = 0.0f;   // acceleration gain (pwm per unit acceleration)

    //PI
    float kP = 0.0f;
    float kI = 0.0f;

    // Output / integral limits 
    float outputMin = -255.0f;
    float outputMax = 255.0f;
    float integralMin = -255.0f;
    float integralMax = 255.0f;
};

struct FFPIDebug {
    float targetVelocity = 0.0f;
    float targetAcceleration = 0.0f;
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
    float compute(float targetVelocity, float targetAcceleration,float actualVelocity, float dt);
    const FFPIDebug& getDebug() const;
    float getIntegral() const;

private:
    static float clamp(float value, float lo, float hi);

    FFPIConfig cfg_;
    float integral_ = 0.0f;
    FFPIDebug lastDebug_;
};

