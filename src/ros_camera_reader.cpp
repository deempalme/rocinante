#include "rocinante/ros_camera_reader.h"

namespace rocinante {
  ROSCameraReader::ROSCameraReader(const std::string &camera_message_name,
                                   const std::string &image_message_name) :
    node_(),
    image_input_(node_.subscribe(image_message_name, 1, &ROSCameraReader::ImageReader, this)),
    camera_input_(node_.subscribe(camera_message_name, 1,
                                  &ROSCameraReader::CameraInfoReader, this)),
    image_message_name_(image_message_name),
    camera_message_name_(camera_message_name),
    initialized_(false),
    total_size_(0),
    // 8 bits is equal to 1 byte and there are at least 3 components for each pixel
    pixel_byte_size_(3)
  {}

  ROSCameraReader::~ROSCameraReader(){
    if(initialized_)
      free(image_.data);
  }

  void ROSCameraReader::Resubscribe(const std::string &image_message_name,
                                    const std::string &camera_message_name){
    image_input_.shutdown();
    image_input_ = node_.subscribe(image_message_name, 1, &ROSCameraReader::ImageReader, this);
    image_message_name_ = image_message_name;
    initialized_ = false;

    camera_input_.shutdown();
    camera_input_ = node_.subscribe(camera_message_name, 1,
                                    &ROSCameraReader::CameraInfoReader, this);
    camera_message_name_ = camera_message_name;
  }

  const torero::ImageFile *ROSCameraReader::CameraImage() const{
    return &image_;
  }

  const torero::CameraInfo *ROSCameraReader::CameraInfo() const{
    return &camera_;
  }

  const std::string &ROSCameraReader::ImageSubscribedMessage(){
    return image_message_name_;
  }

  const std::string &ROSCameraReader::CameraInfoSubscribedMessage(){
    return camera_message_name_;
  }

  boost::signals2::signal<void ()> *ROSCameraReader::SignalUpdate(){
    return &signal_update_;
  }

  // :::::::::::::::::::::::::::::::::::: PRIVATE ::::::::::::::::::::::::::::::::::::

  void ROSCameraReader::ImageReader(const sensor_msgs::Image::ConstPtr &msg){
    // Image properties
    const std::size_t width{static_cast<std::size_t>(msg->width)};
    const std::size_t height{static_cast<std::size_t>(msg->height)};

    if(!initialized_){
      image_.width  = static_cast<int>(msg->width);
      image_.height = static_cast<int>(msg->height);

      // Note that allowed encoding are the following
      if(msg->encoding == sensor_msgs::image_encodings::RGB8){
        image_.components_size = torero::ImageEncoding::RGB;
        image_.is_16bits = false;
        image_.is_inverted = false;
      }else if(msg->encoding == sensor_msgs::image_encodings::BGR8){
        image_.components_size = torero::ImageEncoding::BGR;
        image_.is_16bits = false;
        image_.is_inverted = true;
      }else if(msg->encoding == sensor_msgs::image_encodings::MONO8){
        image_.components_size = torero::ImageEncoding::RED;
        image_.is_16bits = false;
        image_.is_inverted = false;
        pixel_byte_size_ = 1;
      }else if(msg->encoding == sensor_msgs::image_encodings::RGB16){
        image_.components_size = torero::ImageEncoding::RGB;
        image_.is_16bits = true;
        image_.is_inverted = false;
        pixel_byte_size_ = 6;
      }else if(msg->encoding == sensor_msgs::image_encodings::BGR16){
        image_.components_size = torero::ImageEncoding::BGR;
        image_.is_16bits = true;
        image_.is_inverted = true;
        pixel_byte_size_ = 6;
      }else if(msg->encoding == sensor_msgs::image_encodings::MONO16){
        image_.components_size = torero::ImageEncoding::RED;
        image_.is_16bits = true;
        image_.is_inverted = false;
        pixel_byte_size_ = 2;
      }

      // Allocating memory
      total_size_ = pixel_byte_size_ * width * height;
      image_.data = (unsigned char*) malloc(total_size_);
      initialized_ = true;
    }

    // Total image size in bytes
    const std::size_t total_size{ pixel_byte_size_ * width * height };

    if(total_size_ != total_size){
      std::cout << "\n\033[1;41m Error: \033[0;1;38;5;174m \n"
                << "The image's pixel size has a variation.\n"
                << "    Old: " << total_size_ / pixel_byte_size_
                << "  |  New: " << total_size / pixel_byte_size_
                << "\033[0m" << std::endl;
      return;
    }
    // Copying memory
    memcpy((void*)image_.data, (void*)msg->data.data(), total_size_);

    signal_update_();
  }

  void ROSCameraReader::CameraInfoReader(const sensor_msgs::CameraInfo::ConstPtr &msg){

  }
}
