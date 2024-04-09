#include "hardware/arduino_comms.hpp"

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>

namespace arduino_comms
{

  ArduinoComms::ArduinoComms() = default;

  LibSerial::BaudRate ArduinoComms::convert_baud_rate(int baud_rate)
  {
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
  }

  void ArduinoComms::connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    this->timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void ArduinoComms::disconnect()
  {
    serial_conn_.Close();
  }

  bool ArduinoComms::connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string ArduinoComms::send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void ArduinoComms::send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void ArduinoComms::read_encoder_values(int &val_1, int &val_2)
  {
    std::string response = send_msg("e\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
  }

  void ArduinoComms::read_wheels_speed(double &val_1, double &val_2)
  {
    std::string response = send_msg("v\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atof(token_1.c_str());
    val_2 = std::atof(token_2.c_str());
  }
  
  void ArduinoComms::read_lidar_values(int lidar_id, int &direction, float &distance)
  {
    std::string cmd = "t " + std::to_string(lidar_id) + "\r";
    std::string response = send_msg(cmd);
    direction = std::atoi(response.c_str());

    cmd = "p\r";
    response = send_msg(cmd);
    distance = std::atof(response.c_str());
  }

  void ArduinoComms::read_servo_value(int servo_id, int &direction)
  {
    std::stringstream ss;
    ss << "t " << servo_id << "\r";
    std::string response = send_msg(ss.str());

    direction = std::atoi(response.c_str());
  }

  void ArduinoComms::set_servo_value(int servo_id, int direction)
  {
    std::stringstream ss;
    ss << "s " << servo_id << " " << direction << "\r";
    send_msg(ss.str());
  }

  void ArduinoComms::set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "o " << val_1 << " " << val_2 << "\r";  // should be "m " to set vel
    send_msg(ss.str());
  }

  void ArduinoComms::set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }
} // namespace arduino_comms