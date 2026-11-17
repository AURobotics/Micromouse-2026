#pragma once
#include "motor.h"
#include "tim.h"

namespace Test {
inline void motors()
{
    Motor R_motor(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4);
    Motor L_motor(&htim1, TIM_CHANNEL_2, TIM_CHANNEL_1);
    R_motor.init();
    L_motor.init();

    R_motor.move(100);
    L_motor.move(-100);
    HAL_Delay(1000);
    R_motor.move(0);
    L_motor.move(0);

}
void encoders();
void bluetooth();
void IR();
void EEPROM();
void BNO();

}