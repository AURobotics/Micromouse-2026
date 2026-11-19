#pragma once
#include "BNO055.h"
#include "eeprom.h"
#include "i2c.h"
#include "usart.h"

struct imu_utility
{
    static float wrapAngle(float angle)
    {
        while (angle > 180)
            angle -= 360;
        while (angle < -180)
            angle += 360;
        return angle;
    }
    static double angleDiff(double start, double goal)
    {
        // goal  = (goal + 360) % 360.0;
        double diff = fmod(goal - start, 360.0);
        if (diff > 180)
            diff -= 360;
        if (diff < -180)
            diff += 360;
        return diff;
    }

    static bool calibrateBnoAndSave(const imu& bno)
    {
        //TODO use appropriate timing functions
        Calibration_t s{};
        unsigned long start = HAL_GetTick();
        const unsigned long TIMEOUT_MS = 120000;
        uint8_t buf[64];
        int n;
        printf("BNO Calibration...\n");
        while (true)
        {
            bno.calibration_status(s);
            printf("BNO Calibration: sys=%d gyro=%d accel=%d  mag=%d\r\n", s.sys, s.gyro,
                         s.accel, s.mag);
            if (/*s.sys == 3 &&*/ s.gyro == 3 && s.accel == 3 /*&& s.mag == 3*/)
                break;

            if (HAL_GetTick() - start > TIMEOUT_MS)
            {
                printf("BNO calibration time out\r\n");
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        CalibProfile_t p{};
        bno.getOffsets(p);
        uint8_t magic_value = EEPROM_MAGIC;
        if (writeCalibration(&hi2c1, EEPROM_ADDR_BNO_OFFSETS, p.data, 22))
        {
            writeCalibration(&hi2c1, EEPROM_ADDR_BNO_VALID, &magic_value, 1);
            n = snprintf((char*)buf, sizeof buf, "BNO calibration saved to EEPROM :)\r\n");
            HAL_UART_Transmit(&huart2, buf, n, 20);
        }
        else
        {
            n = snprintf((char*)buf, sizeof buf, "Saving bno calibration to eeprom failed :(\r\n");
            HAL_UART_Transmit(&huart2, buf, n, 20);
        }

        return true;
    }

    static bool loadBnoCalibration(const imu& bno)
    {
        uint8_t magic_value{};
        readCalibration(&hi2c1, 0, &magic_value, 1);
        if (magic_value == EEPROM_MAGIC)
        {
            CalibProfile_t p{};
            readCalibration(&hi2c1, 5, p.data, 22);
            bno.setOffsets(p);
            return true;
        }
        return false;
    }
};