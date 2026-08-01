#pragma once

class ButterworthIIR{
    private:
        float b0, b1, b2, a1, a2;
        float x1, x2, y1, y2;

    public:
    ButterworthIIR()=default;
    void reset();
    void init(float cutoff_freq, float sample_rate);
/* 
transfer function: H(s) = 1/(s^2 + sqrt(2)*s + 1)
transorm IIR filter to discrete time using bilinear transform
*/
    inline float filter(float x){  //x -->input  , y -->output
        float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
        return y;
    }

};