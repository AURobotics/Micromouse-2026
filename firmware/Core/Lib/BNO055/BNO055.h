#pragma once
#include "stm32f4xx_hal.h"  
#include "FreeRTOS.h"       
#include "task.h" 
#include "BNO055_registers.h"
// enum class sensor_t { GYRO, ACCEL, MAG };
struct Calibration_t {
    uint8_t sys = 0;
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;
};

struct CalibProfile_t{
  uint8_t data[22]; //offset data
};

struct vec_3 {
    float vec[3];
    float& x() { return vec[0]; }
    float& y() { return vec[1]; }
    float& z() { return vec[2]; }
    const float& x() const { return vec[0]; }
    const float& y() const { return vec[1]; }
    const float& z() const { return vec[2]; }
};

struct vec_4 {
    float vec[4];
    float& w() { return vec[0]; }
    float& y() { return vec[1]; }
    float& z() { return vec[2]; }
    float& x() { return vec[3]; }
    const float& w() const { return vec[0]; }
    const float& y() const { return vec[1]; }
    const float& z() const { return vec[2]; }
    const float& x() const { return vec[3]; }
};


struct imu {
public:
    imu(I2C_HandleTypeDef* hi2c, const uint8_t address);
    void init();
    void set_mode(operation_mode mode) const;
    void remap_axis(uint8_t config) const; // 0 to 7
    void remap_axis(::remap_axis axis, remap_sign sign) const;
    void remap_axis(::remap_axis remapped_z,::remap_axis remapped_y,::remap_axis remapped_x,::remap_sign sign_z,::remap_sign sign_y,::remap_sign sign_x)const;
    void calibration_status(Calibration_t& s) const;
    void getOffsets(CalibProfile_t& p) const;
    void setOffsets(const CalibProfile_t& p) const;
    // void set_units_metric() const;
    // void set_units_default() const;
    int8_t temperature() const;
    vec_3 acceleration() const;
    vec_3 linear_acceleration() const;
    vec_3 gyro() const;
    vec_3 mag() const;
    vec_3 euler() const;
    vec_4 quaternion() const;
    bool isConnected() const;
    
    
private:
    
    I2C_HandleTypeDef hi2c;
    uint8_t address;

    uint8_t read_register(uint8_t reg, uint8_t* buffer, uint8_t len = 8) const;
    void write_register(uint8_t reg, uint8_t val) const;
};