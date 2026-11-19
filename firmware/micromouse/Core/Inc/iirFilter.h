#pragma once
#include "cmath"

// 2 pole Butterworth
// H(s) = 1/(s^2 + sqrt(2)*s + 1)

class ButterworthIIR
{
    float b0{}, b1{}, b2{}, a1{}, a2{};
    float x1{}, x2{}, y1{}, y2{};

public:
    explicit constexpr ButterworthIIR(float cutoff_freq, float sample_rate)
    {
        if (cutoff_freq <= 0.0f || sample_rate <= 0.0f || cutoff_freq >= 0.5f * sample_rate)
        {
            b0 = b1 = b2 = a1 = a2 = 0.0f;
            return;
        }

        const float kPi = 3.14159265358979323846f;
        const float ksqrt2 = 1.41421356237309504880f;

        float ohm = std::tan(kPi * cutoff_freq / sample_rate);
        float ohm_2 = ohm * ohm;

        float c = 1.0f + (ksqrt2 * ohm) + ohm_2;
        float c_inv = 1.0f / c;

        b0 = ohm_2 * c_inv;
        b1 = 2.0f * b0;
        b2 = b0;
        a1 = 2.0f * (ohm_2 - 1.0f) * c_inv;
        a2 = (1.0f - (ksqrt2 * ohm) + ohm_2) * c_inv;
    }

    ButterworthIIR() = delete;

    // hehe delete copy operator 3shan howa keda
    ButterworthIIR(const ButterworthIIR&) = delete;
    ButterworthIIR& operator=(const ButterworthIIR&) = delete;

    void reset() { x1 = x2 = y1 = y2 = 0.0f; }

    /*
    transfer function: H(s) = 1/(s^2 + sqrt(2)*s + 1)
    transform IIR filter to discrete time using bilinear transform
    */
    float filter(float x)
    { // x -->input  , y -->output
        float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
        return y;
    }
};
