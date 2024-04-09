#ifndef HARDWARE__ARDUINO_COMMS_HPP
#define HARDWARE__ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>

namespace arduino_comms
{
class ArduinoComms
{
private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;

public:

  ArduinoComms();

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void disconnect();
  bool connected() const;
  std::string send_msg(const std::string &msg_to_send, bool print_output);
  void send_empty_msg();

  LibSerial::BaudRate convert_baud_rate(int baud_rate);
  
  void read_encoder_values(int &val_1, int &val_2);
  void read_wheels_speed(double &val_1, double &val_2);
  void read_lidar_values(int lidar_id, int &direction, float &distance);
  void read_servo_value(int servo_id, int &direction);
  
  void set_servo_value(int servo_id, int direction);
  void set_motor_values(int val_1, int val_2);
  void set_pid_values(int k_p, int k_d, int k_i, int k_o);
};

} // namespace arduino_comms

#endif // HARDWARE__ARDUINO_COMMS_HPP