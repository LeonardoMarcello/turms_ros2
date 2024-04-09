#include <stdio.h>
#include <string.h>
#include <wiringPiI2C.h>

#ifndef HARDWARE__MPU9150_HPP
#define HARDWARE__MPU9150_HPP
namespace mpu9150{

class MPU9150{     
    
  private:
    int i2c_conn_;
    int timeout_ms_;

  public:
    void connect(const int i2c_device, int32_t timeout_ms);
    void start_spi();
    void read_registers(uint8_t reg, uint8_t *buf, uint16_t len);

    void read_raw_accel(double &accel_x, double &accel_y, double &accel_z);
    void read_raw_gyro(double &gyro_x, double &gyro_y, double &gyro_z);
    void read_raw_mag(double &mag_x, double &mag_y, double &mag_z);
    
    void calculate_angles_from_accel(double &roll, double &pitch, double &yaw);
};
} // namespace mpu9150
#endif // HARDWARE__MPU9150_HPP