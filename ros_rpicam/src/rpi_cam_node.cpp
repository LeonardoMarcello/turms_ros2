#include "ros_rpicam/rpi_cam_node.hpp"

using namespace libcamera;

namespace rpicamera{
    
rpiCamNode::rpiCamNode(const rclcpp::NodeOptions & options)
: Node("rpi_cam", options)
{
  declare_cam_parameters();

  width_ = get_parameter("image_width").as_int();
  height_ = get_parameter("image_height").as_int();
  fps_ = get_parameter("framerate").as_double();
  frame_id_ = get_parameter("camera_frame_id").as_string();
  format_str = this->get_parameter("pixel_format").as_string();

  image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, get_parameter("camera_name").as_string());

  setup_camera();

  RCLCPP_INFO(get_logger(), "libcamera usb_cam-compatible node started");
}

rpiCamNode::~rpiCamNode(){
    camera_->stop();
    camera_->release();
    cm_->stop();
}

void rpiCamNode::declare_cam_parameters(){
    declare_parameter("camera_name", "camera");
    declare_parameter("camera_frame_id", "camera");

    declare_parameter("image_width", 640);
    declare_parameter("image_height", 480);
    declare_parameter("framerate", 30.0);

    declare_parameter("pixel_format", "rgb8");

    declare_parameter("auto_exposure", true);
    declare_parameter("exposure", 100);
    declare_parameter("gain", 1.0);

    // usb_cam compatibility (ignored but accepted)
    declare_parameter("brightness", -1);
    declare_parameter("contrast", -1);
    declare_parameter("saturation", -1);
    declare_parameter("sharpness", -1);
}


void rpiCamNode::setup_camera() {
    cm_ = std::make_unique<CameraManager>();
    cm_->start();

    if (cm_->cameras().empty()) throw std::runtime_error("No cameras found");
    camera_ = cm_->cameras()[0];
    camera_->acquire();

    config_ = camera_->generateConfiguration({StreamRole::Viewfinder});
    auto &cfg = config_->at(0);
    cfg.size.width = width_;
    cfg.size.height = height_;
    // get pixel libcamera formats
    if (format_str == "rgb8") cfg.pixelFormat = formats::RGB888;
    else if (format_str == "bgr8") cfg.pixelFormat = formats::BGR888;
    else if (format_str == "yuyv") cfg.pixelFormat = formats::YUYV;
    else if (format_str == "nv12") cfg.pixelFormat = formats::NV12;
    else if (format_str == "mono8") cfg.pixelFormat = formats::R8;
    else{
      // list available type and set bgr8
      const StreamFormats &formats = cfg.formats();
      for (auto const& fmt : formats.pixelformats()) {
          RCLCPP_INFO(this->get_logger(), "Supported Format: %s", fmt.toString().c_str());
      }
      cfg.pixelFormat =  formats::BGR888;
    }

    
    // Set framerate (Duration is in microseconds: 1,000,000 / FPS)
    int64_t frame_time = 1000000 / fps_;

    // Check and apply desired configuration
    libcamera::CameraConfiguration::Status status = config_->validate();
    if (status == libcamera::CameraConfiguration::Invalid) {
        RCLCPP_ERROR(this->get_logger(), "Camera configuration is invalid and could not be corrected.");
        throw std::runtime_error("Fatal libcamera configuration error");
    }
    if (status == libcamera::CameraConfiguration::Adjusted) {
        RCLCPP_WARN(this->get_logger(), "Camera configuration was ADJUSTED by the hardware.");
        
        // Check if the resolution changed
        if (cfg.size.width != (unsigned int)width_ || cfg.size.height != (unsigned int)height_) {
            RCLCPP_WARN(this->get_logger(), "Resolution changed from %dx%d to %dx%d",
                        width_, height_, cfg.size.width, cfg.size.height);
            // Update your class members so your ROS publisher uses the correct size
            width_ = cfg.size.width;
            height_ = cfg.size.height;
        }
        // Check if the pixel format changed
        RCLCPP_INFO(this->get_logger(), "Final Pixel Format: %s", cfg.pixelFormat.toString().c_str());
    }else {
        RCLCPP_INFO(this->get_logger(), "Camera configuration validated successfully.");
    }

    camera_->configure(config_.get());

    // 3. Setup Controls (Exposure, Gain, etc.)
    ControlList controls;
    
    if (this->get_parameter("auto_exposure").as_bool()) {
        controls.set(controls::AeEnable, true);
    } else {
        controls.set(controls::AeEnable, false);
        // ExposureTime is in microseconds
        controls.set(controls::ExposureTime, this->get_parameter("exposure").as_int());
        controls.set(controls::AnalogueGain, (float)this->get_parameter("gain").as_double());
    }

    // 4. Finalize
    stream_ = cfg.stream();
    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
    allocator_->allocate(stream_);

    setup_requests();

    camera_->requestCompleted.connect(this, &rpiCamNode::publish_frame);
    
    // Start camera with the control list
    camera_->start(&controls);

    for (auto &req : requests_)
        camera_->queueRequest(req.get());
}

void rpiCamNode::setup_requests(){
  for (auto &buffer : allocator_->buffers(stream_)) {
    auto req = camera_->createRequest();
    req->addBuffer(stream_, buffer.get());
    requests_.push_back(std::move(req));
  }
}


void rpiCamNode::publish_frame(Request *request)
{
  if (request->status() != Request::RequestComplete)
    return;

  auto buffer = request->buffers().begin()->second;
  const auto &plane = buffer->planes()[0];

  void *data = mmap(
    nullptr,
    plane.length,
    PROT_READ,
    MAP_SHARED,
    plane.fd.get(),
    0);

  cv::Mat image(
    height_,
    width_,
    CV_8UC3,
    data
  );

  auto msg = cv_bridge::CvImage(
    std_msgs::msg::Header(),
    "rgb8",
    image
  ).toImageMsg();

  msg->header.stamp = now();
  msg->header.frame_id = frame_id_;

  image_pub_->publish(*msg);

  auto info = cinfo_->getCameraInfo();
  info.header = msg->header;
  info_pub_->publish(info);

  munmap(data, plane.length);

  request->reuse(Request::ReuseBuffers);
  camera_->queueRequest(request);
}
}//namespace rpicamera
