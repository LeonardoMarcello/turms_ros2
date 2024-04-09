#include "hardware/mpu9150.hpp"

#include <iostream>
#include <errno.h>
#include <wiringPiI2C.h>

namespace mpu9150{

MPU9150::MPU9150() = default;

void MPU9150::connect(const int i2c_device, int32_t timeout_ms){
    int fd = wiringPiI2CSetup(i2c_device);
    std::cout << "Init result: "<< fd << std::endl;
    this->i2c_conn_ = fd;
}
void read_registers(uint8_t reg, int &buf){
    buf = wiringPiI2CReadReg16(this->i2c_conn_, reg);
}

void MPU9150::read_raw_accel(double &accel_x, double &accel_y, double &accel_z){

}
void MPU9150::read_raw_gyro(double &gyro_x, double &gyro_y, double &gyro_z){

}
void MPU9150::read_raw_mag(double &mag_x, double &mag_y, double &mag_z){

}
    
void MPU9150::calculate_angles_from_accel(double &roll, double &pitch, double &yaw){

}
} // namespace mpu9150

