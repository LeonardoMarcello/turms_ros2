#ifndef ARDUINO_SERVOMOTOR_HPP
#define ARDUINO_SERVOMOTOR_HPP

#include <string>

class Servo
{
    public:

    std::string name = "";
    double cmd_pos = 0;
    double pos = 0;
    double ping = -1;

    Servo() = default;

    Servo(const std::string &servo_name)
    {
      setup(servo_name);
    }

    
    void setup(const std::string &servo_name)
    {
      name = servo_name;
    }

};


#endif // ARDUINO_SERVOMOTOR_HPP