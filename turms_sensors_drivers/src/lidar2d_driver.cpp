#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "hardware/arduino_comms.hpp"
//#include "hardware/include/hardware/arduino_comms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "turms_msgs/msg/obstacle.hpp"

using namespace std::chrono_literals;
using namespace arduino_comms;

class Lidar2dDriver : public rclcpp::Node
{

  struct Config{
    std::string name = "lidar2d";
    std::string frame = "lidar2d";
    std::string device = "/dev/ttyUSB0";
    float loop_rate = 20.0;
    int baud_rate = 56700;
    int timeout_ms = 5000;
  };

  public:
    Lidar2dDriver() // constructor initialization
    :Node("lidar2d"), count_(0)     // naming_node, intialize count_ to 0
    {          
        // parameters declaration
        this->declare_parameter("frame", "lidar2d");              // node rate [Hertz]
        this->declare_parameter("rate", 20.0);              // node rate [Hertz]
        this->declare_parameter("device", "/dev/ttyUSB0");   // serial port name  
        this->declare_parameter("baudrate", 57600);         // serial port baudrate  
        this->declare_parameter("timeout", 5000);           // serial port timeout [ms] 
        
        cfg_.device = this->get_parameter("frame").as_string();
        cfg_.loop_rate = this->get_parameter("rate").as_double();
        cfg_.device = this->get_parameter("device").as_string();
        cfg_.baud_rate = this->get_parameter("baudrate").as_int();
        cfg_.timeout_ms = this->get_parameter("timeout").as_int();
        
        
        // starting serial communication
        RCLCPP_INFO(this->get_logger(), "Starting serial communication with device: '%s'", cfg_.device.c_str());
        try{
            comms_.connect(cfg_.device, cfg_.baud_rate,cfg_.timeout_ms);
            RCLCPP_INFO(this->get_logger(), "Communication successfully started");
        }
        catch (const LibSerial::OpenFailed&)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot open communication with '%s'", cfg_.device.c_str());
            throw;
        }

        // publisher and timer setup
        publisher_ = this->create_publisher<turms_msgs::msg::Obstacle>("obstacle", 10);
        timer_ = this->create_wall_timer((1/cfg_.loop_rate)*1s, 
                                        std::bind(&Lidar2dDriver::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Node successfully started");
    }

  private:
    void timer_callback(){
      int direction = -1;
      float distance = -1.0;
      comms_.read_lidar_values(0, direction, distance);

      auto message = turms_msgs::msg::Obstacle();
      message.header.frame_id = cfg_.frame.c_str();
      message.header.stamp.sec = rclcpp::Clock{}.now().seconds();
      message.header.stamp.nanosec = rclcpp::Clock{}.now().nanoseconds();
      message.direction = static_cast<float>(direction-90);
      message.distance = static_cast<float>(distance);

      RCLCPP_INFO(this->get_logger(), "Publishing: %.2f cm at %.0f degrees", message.distance, message.direction);
      publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<turms_msgs::msg::Obstacle>::SharedPtr publisher_;
    ArduinoComms comms_;
    Config cfg_;
    size_t count_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  try{
    rclcpp::spin(std::make_shared<Lidar2dDriver>());
  }
  catch(const LibSerial::OpenFailed&){
    rclcpp::shutdown();
    return -1;
  }
  rclcpp::shutdown();
  return 0;
}