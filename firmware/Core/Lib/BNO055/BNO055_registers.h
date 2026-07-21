#pragma once
#include "bits/stdc++.h"



enum class operation_mode : uint8_t
{
    CONFIG = 0X00,
    ACCONLY = 0X01,
    MAGONLY = 0X02,
    GYRONLY = 0X03,
    ACCMAG = 0X04,
    ACCGYRO = 0X05,
    MAGGYRO = 0X06,
    AMG = 0X07,
    IMU = 0X08,
    COMPASS = 0X09,
    M4G = 0X0A,
    NDOF_FMC_OFF = 0X0B,
    NDOF = 0X0C
};

// inline std::function<std::optional<std::vector<double>>(imu&)> operation_mode_functions[13] = {
//     [](imu& instance) { return instance.config(); },
//     [](imu& instance) { return instance.acconly(); },
//     [](imu& instance) { return instance.magonly(); },
//     [](imu& instance) { return instance.gyroonly(); },
//     [](imu& instance) { return instance.accmag(); },
//     [](imu& instance) { return instance.accgyro(); },
//     [](imu& instance) { return instance.maggyro(); },
//     [](imu& instance) { return instance.amg(); },
//     [](imu& instance) { return instance.imu_mode(); },
//     [](imu& instance) { return instance.compass(); },
//     [](imu& instance) { return instance.m4g(); },
//     [](imu& instance) { return instance.ndof_fmc_off(); },
//     [](imu& instance) { return instance.ndof(); }
// };

inline constexpr std::pair<uint8_t, uint8_t> axis_remap_table[] = {
    {0x21, 0x04}, // P0
    {0x24, 0x00}, // P1`
    {0x24, 0x06}, // P2
    {0x21, 0x02}, // P3
    {0x24, 0x03}, // P4
    {0x21, 0x01}, // P5
    {0x21, 0x07}, // P6
    {0x24, 0x05}  // P7
};

enum class remap_axis : uint8_t
{
    x_axis = 0x00,
    y_axis = 0x01,
    z_axis = 0x02,
};

enum class remap_sign : uint8_t
{
    positive = 0x00,
    negative = 0x01,
};

namespace imu_registers
{
    struct id {
        static constexpr uint8_t PAGE_ID = 0X07;
        static constexpr uint8_t CHIP_ID = 0x00;
        static constexpr uint8_t ACCEL_REV_ID = 0x01;
        static constexpr uint8_t MAG_REV_ID = 0x02;
        static constexpr uint8_t GYRO_REV_ID = 0x03;
        static constexpr uint8_t SW_REV_ID_LSB = 0x04;
        static constexpr uint8_t SW_REV_ID_MSB = 0x05;
        static constexpr uint8_t BL_REV_ID = 0X06;
    };

    struct sensor_data {
        static constexpr uint8_t ACCEL_X_LSB = 0X08;
        static constexpr uint8_t ACCEL_X_MSB = 0X09;
        static constexpr uint8_t ACCEL_Y_LSB = 0X0A;
        static constexpr uint8_t ACCEL_Y_MSB = 0X0B;
        static constexpr uint8_t ACCEL_Z_LSB = 0X0C;
        static constexpr uint8_t ACCEL_Z_MSB = 0X0D;

        static constexpr uint8_t MAG_X_LSB = 0X0E;
        static constexpr uint8_t MAG_X_MSB = 0X0F;
        static constexpr uint8_t MAG_Y_LSB = 0X10;
        static constexpr uint8_t MAG_Y_MSB = 0X11;
        static constexpr uint8_t MAG_Z_LSB = 0X12;
        static constexpr uint8_t MAG_Z_MSB = 0X13;

        static constexpr uint8_t GYRO_X_LSB = 0X14;
        static constexpr uint8_t GYRO_X_MSB = 0X15;
        static constexpr uint8_t GYRO_Y_LSB = 0X16;
        static constexpr uint8_t GYRO_Y_MSB = 0X17;
        static constexpr uint8_t GYRO_Z_LSB = 0X18;
        static constexpr uint8_t GYRO_Z_MSB = 0X19;

        static constexpr uint8_t EUL_X_LSB = 0X1A;
        static constexpr uint8_t EUL_X_MSB = 0X1B;
        static constexpr uint8_t EUL_Y_LSB = 0X1C;
        static constexpr uint8_t EUL_Y_MSB = 0X1D;
        static constexpr uint8_t EUL_Z_LSB = 0X1E;
        static constexpr uint8_t EUL_Z_MSB = 0X1F;

        static constexpr uint8_t QUAT_W_LSB = 0X20;
        static constexpr uint8_t QUAT_W_MSB = 0X21;
        static constexpr uint8_t QUAT_X_LSB = 0X22;
        static constexpr uint8_t QUAT_X_MSB = 0X23;
        static constexpr uint8_t QUAT_Y_LSB = 0X24;
        static constexpr uint8_t QUAT_Y_MSB = 0X25;
        static constexpr uint8_t QUAT_Z_LSB = 0X26;
        static constexpr uint8_t QUAT_Z_MSB = 0X27;

        static constexpr uint8_t LINEAR_ACCEL_X_LSB = 0X28;
        static constexpr uint8_t LINEAR_ACCEL_X_MSB = 0X29;
        static constexpr uint8_t LINEAR_ACCEL_Y_LSB = 0X2A;
        static constexpr uint8_t LINEAR_ACCEL_Y_MSB = 0X2B;
        static constexpr uint8_t LINEAR_ACCEL_Z_LSB = 0X2C;
        static constexpr uint8_t LINEAR_ACCEL_Z_MSB = 0X2D;

        static constexpr uint8_t GRAVITY_X_LSB = 0X2E;
        static constexpr uint8_t GRAVITY_X_MSB = 0X2F;
        static constexpr uint8_t GRAVITY_Y_LSB = 0X30;
        static constexpr uint8_t GRAVITY_Y_MSB = 0X31;
        static constexpr uint8_t GRAVITY_Z_LSB = 0X32;
        static constexpr uint8_t TEMP = 0X34;
    };

    struct status {
        static constexpr uint8_t CALIB = 0X35;
        static constexpr uint8_t SELFTEST_RESULT = 0X36;
        static constexpr uint8_t INTR_ADDR = 0X37;
        static constexpr uint8_t SYS_CLK = 0X38;
        static constexpr uint8_t SYS_STAT = 0X39;
        static constexpr uint8_t SYS_ERR = 0X3A;
    };

    struct unit {
        static constexpr uint8_t UNIT_SEL = 0X3B;
    };

    struct mode {
        static constexpr uint8_t OPR_MODE = 0X3D;
        static constexpr uint8_t PWR_MODE = 0X3E;
        static constexpr uint8_t SYS_TRIGGER = 0X3F;
        static constexpr uint8_t TEMP_SOURCE = 0X40;
    };

    struct axis_remap {
        static constexpr uint8_t REMAP_AXIS = 0X41;
        static constexpr uint8_t REMAP_SIGN = 0X42;
    };

    struct offset {
        static constexpr uint8_t ACCEL_X_LSB = 0X55;
        static constexpr uint8_t ACCEL_X_MSB = 0X56;
        static constexpr uint8_t ACCEL_Y_LSB = 0X57;
        static constexpr uint8_t ACCEL_Y_MSB = 0X58;
        static constexpr uint8_t ACCEL_Z_LSB = 0X59;
        static constexpr uint8_t ACCEL_Z_MSB = 0X5A;

        static constexpr uint8_t MAG_X_LSB = 0X5B;
        static constexpr uint8_t MAG_X_MSB = 0X5C;
        static constexpr uint8_t MAG_Y_LSB = 0X5D;
        static constexpr uint8_t MAG_Y_MSB = 0X5E;
        static constexpr uint8_t MAG_Z_LSB = 0X5F;
        static constexpr uint8_t MAG_Z_MSB = 0X60;

        static constexpr uint8_t GYRO_X_LSB = 0X61;
        static constexpr uint8_t GYRO_X_MSB = 0X62;
        static constexpr uint8_t GYRO_Y_LSB = 0X63;
        static constexpr uint8_t GYRO_Y_MSB = 0X64;
        static constexpr uint8_t GYRO_Z_LSB = 0X65;
        static constexpr uint8_t GYRO_Z_MSB = 0X66;
    };

    struct radius {
        static constexpr uint8_t ACCEL_LSB = 0X67;
        static constexpr uint8_t ACCEL_MSB = 0X68;
        static constexpr uint8_t MAG_LSB = 0X69;
        static constexpr uint8_t MAG_MSB = 0X6A;
    };
} // namespace imu_registers