#include "BNO055.h"
#include "BNO055_registers.h"
#include <cstring>

//switching from CONFIGmode --> any other mode  === 7ms
//switching from any mode --> CONFIGmode === 19ms

imu::imu(I2C_HandleTypeDef* hi2c, const uint8_t address) :
    hi2c(hi2c), address(address) {
    /* No hardware calls here hi2c initialized elsewhere
       (MX_I2Cx_Init, called from CubeMX-generated init code) before this
       object is used.
       
       removed hardware calls w keda fy init function
       3ashan ne initialize bno gwa el setup 3ashan the code crashes
       somehow if i initialized it abl el setup
    */
}

void imu::init()
{
    int found_count = 0;
    for (uint8_t addr7 = 0x08; addr7 <= 0x77; ++addr7) {
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, addr7 << 1, 2, 5);
        if (status == HAL_OK) {
            printf("  Found device at 0x%02X\r\n", addr7);
            ++found_count;
        }
        // small gap between probes to avoid hammering the bus back-to-back
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    while (!isConnected()) {
        //TODO:
        //HAL function that checks if i2c is connected
        vTaskDelay(pdMS_TO_TICKS(850));
        printf("bno not connected");
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, address << 1, 5, 20);
        printf("status: %d\n",status);
    }
    this->set_mode(operation_mode::CONFIG);
    vTaskDelay(pdMS_TO_TICKS(20));
    // celsius, radians, rad/s, m/s2
    constexpr uint8_t unit = 1 << 2 | 1 << 1;
    write_register(imu_registers::unit::UNIT_SEL, unit);
    vTaskDelay(pdMS_TO_TICKS(10));
    this->remap_axis(remap_axis::z_axis, remap_axis::y_axis, remap_axis::x_axis,remap_sign::positive,remap_sign::positive,remap_sign::positive);
    /*imu mounted horizontally*/
    /*
    imu is mounted vertically
    native_z --> x-axis
    native_y --> y-axis
    native_x --> negative z-axis*/
    vTaskDelay(pdMS_TO_TICKS(10));
    set_mode(operation_mode::NDOF);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void imu::set_mode(operation_mode mode) const {
    write_register(imu_registers::mode::OPR_MODE, static_cast<uint8_t>(mode));
}

void imu::remap_axis(const uint8_t config) const {
    if (config > 7)
        return;
    remap_axis(static_cast<::remap_axis>(axis_remap_table[config].first),
               static_cast<remap_sign>(axis_remap_table[config].second));
}

void imu::remap_axis(::remap_axis axis, remap_sign sign) const {
    using ax_rem = imu_registers::axis_remap;
    write_register(ax_rem::REMAP_AXIS, static_cast<uint8_t>(axis));
    write_register(ax_rem::REMAP_SIGN, static_cast<uint8_t>(sign));
}

void imu::remap_axis(::remap_axis remapped_z,::remap_axis remapped_y,::remap_axis remapped_x,::remap_sign sign_z,::remap_sign sign_y,::remap_sign sign_x)const{
    uint8_t axis = static_cast<uint8_t>(remapped_z)<<4 | 
                   static_cast<uint8_t>(remapped_y)<<2 | 
                   static_cast<uint8_t>(remapped_x);

    uint8_t sign = static_cast<uint8_t>(sign_x)<<2 |
                   static_cast<uint8_t>(sign_y)<<1 | 
                   static_cast<uint8_t>(sign_z);

    using ax_rem = imu_registers::axis_remap;
    write_register(ax_rem::REMAP_AXIS, static_cast<uint8_t>(axis));
    write_register(ax_rem::REMAP_SIGN, static_cast<uint8_t>(sign));
}


void imu::calibration_status(Calibration_t& s) const {
    uint8_t calib = 0;
    read_register(imu_registers::status::CALIB, &calib, 1);
    s.sys = calib >> 6 & 0x03;
    s.gyro = calib >> 4 & 0x03;
    s.accel = calib >> 2 & 0x03;
    s.mag = calib >> 0 & 0x03;
}

void imu::getOffsets(CalibProfile_t& p) const {
    set_mode(operation_mode::CONFIG);
    vTaskDelay(pdMS_TO_TICKS(20));
    read_register(imu_registers::offset::ACCEL_X_LSB, p.data, 22);
    set_mode(operation_mode::NDOF);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void imu::setOffsets(const CalibProfile_t& p) const {
    set_mode(operation_mode::CONFIG);
    vTaskDelay(pdMS_TO_TICKS(20));
    uint8_t data[22];
    memcpy(data, p.data, 22);
    // for (int i = 0; i < 22; ++i)
    //     write_register(imu_registers::offset::ACCEL_X_LSB + i, p.data[i]);
    HAL_I2C_Mem_Write(hi2c, address<<1, imu_registers::offset::ACCEL_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 22, 10);
    set_mode(operation_mode::NDOF);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// void imu::set_units_metric() const {
//     write_register(imu_registers::unit::UNIT_SEL, 0x00);
// }
//
// void imu::set_units_default() const {
//     write_register(imu_registers::unit::UNIT_SEL, 0x80);
// }

int8_t imu::temperature() const {
    uint8_t t;
    read_register(imu_registers::sensor_data::TEMP, &t, 1);
    return static_cast<int8_t>(t);
}

vec_3 imu::acceleration() const {
    vec_3 v;
    uint8_t buf[6] = {};
    read_register(imu_registers::sensor_data::ACCEL_X_LSB, buf, 6);
    for (int i = 0; i < 3; ++i) {
        v.vec[i] = static_cast<int16_t>(buf[2 * i + 1] << 8 | buf[2 * i]) /
            100.0f; // scale: 1 LSB = 1 mg = 0.01 m/s^2
    }
    return v;
}

vec_3 imu::linear_acceleration() const {
    vec_3 v;
    uint8_t buf[6] = {};
    read_register(imu_registers::sensor_data::LINEAR_ACCEL_X_LSB , buf, 6);
    for (int i = 0; i < 3; ++i)
        v.vec[i] = static_cast<int16_t>(buf[2*i+1] << 8 | buf[2*i]) / 100.0f;
    return v;
}

vec_3 imu::gyro() const {
    vec_3 v;
    uint8_t buf[6] = {};
    read_register(imu_registers::sensor_data::GYRO_X_LSB, buf, 6);
    for (int i = 0; i < 3; ++i)
        v.vec[i] =
            static_cast<int16_t>(buf[2 * i + 1] << 8 | buf[2 * i]) / 900.0f;
    return v;
}

vec_3 imu::mag() const {
    vec_3 v;
    uint8_t buf[6] = {};
    read_register(imu_registers::sensor_data::MAG_X_LSB, buf, 6);
    for (int i = 0; i < 3; ++i)
        v.vec[i] =
            static_cast<int16_t>(buf[2 * i + 1] << 8 | buf[2 * i]) / 16.0f;
    return v;
}

/**
 * x, y, z
 */
vec_3 imu::euler() const {
    vec_3 v;
    uint8_t buf[6] = {};
    read_register(imu_registers::sensor_data::EUL_X_LSB, buf, 6);
    for (int i = 0; i < 3; ++i)
        v.vec[i] =
            static_cast<int16_t>(buf[2 * i + 1] << 8 | buf[2 * i]) / 16.0f;
    return v;
}

/**
 * w, x, y, z
 */
vec_4 imu::quaternion() const {
    vec_4 v;
    using imu_registers::sensor_data;
    uint8_t buf[8];
    read_register(sensor_data::QUAT_W_LSB, buf, 8);
    for (int i = 0; i < 4; ++i)
        v.vec[i] =
            static_cast<int16_t>(buf[2 * i + 1] << 8 | buf[2 * i]) / 16384.0f;
    return v;
}


HAL_StatusTypeDef imu::write_register(const uint8_t reg, const uint8_t val) const {
    uint8_t data = val;
    return HAL_I2C_Mem_Write(imu::hi2c, address << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);//TODO: i2c timeout?
}

uint8_t imu::read_register(const uint8_t reg, uint8_t* buffer, const uint8_t len) const {
    return HAL_I2C_Mem_Read(imu::hi2c, address << 1, reg,I2C_MEMADD_SIZE_8BIT,buffer, len, 10) == HAL_OK? len:0;
}

bool imu::isConnected() const {
    uint8_t id = 0;
    read_register(imu_registers::id::CHIP_ID, &id, 1);  
    return id == 0xA0;
}