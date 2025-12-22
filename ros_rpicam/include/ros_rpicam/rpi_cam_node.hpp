#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer_allocator.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sys/mman.h>

namespace rpicamera{
class rpiCamNode : public rclcpp::Node{

public:
    explicit rpiCamNode(const rclcpp::NodeOptions & options);
    ~rpiCamNode();

private:
    /* ROS */
    void declare_cam_parameters();
    void publish_frame(libcamera::Request *request);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    /* libcamera */
    void setup_camera();
    void setup_requests();

    std::unique_ptr<libcamera::CameraManager> cm_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;

    libcamera::Stream * stream_ = nullptr;

    /* Parameters */
    int width_;
    int height_;
    double fps_;
    std::string format_str;
    std::string frame_id_;
};
}// namspace rpicamera