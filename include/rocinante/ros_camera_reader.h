#ifndef ROCINANTE_CAMERA_READER_H
#define ROCINANTE_CAMERA_READER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "rocinante/rocinante_type_definitions.h"

#include <cstring>
#include <iostream>
#include <string>
#include <vector>
// signals and slots
#include <boost/bind.hpp>
#include <boost/signals2.hpp>

namespace rocinante {
  class ROSCameraReader
  {
  public:
    /*
     * ### Constructor
     *
     * This function will create the **ROS camera reader** *class*, you need to define
     * the ROS message name and the type of data input. The data `type` must coincide with
     * the `typename T`, see [[data types|ROS-laser-reader#data-structure]] to see
     * which `type` you must select.
     *
     * **Arguments**
     * {const std::string} message_name = ROS message name as listed at console using
     * `rostopic list` command.
     * {const rocinante::CameraSensorType} sensor_type = Type of ROS message.
     *
     * **Errors**
     * This will print a warning message in console if the `typename T` and `type`
     * do not match in the type of output.
     *
     */
    ROSCameraReader(const std::string &camera_message_name,
                    const std::string &image_message_name);
    ~ROSCameraReader();
    /*
     * ### Changing the ROS topic (message)
     *
     * This will change the read message, the new message must be the same type than the previous.
     *
     * **Arguments**
     * {const std::string} message_name = ROS message name as listed at console using
     * `rostopic list` command.
     *
     */
    void Resubscribe(const std::string &image_message_name,
                     const std::string &camera_message_name);
    /*
     * ### Obtaining the address to the data vector
     *
     * This function will return the address to the vector containing all the read data.
     *
     * **Returns**
     * {const std::vector<T> *const} Returns the address of the vector containing the data.
     *
     */
    const torero::ImageFile *CameraImage() const;
    const torero::CameraInfo *CameraInfo() const;
    /*
     * ### Obtaining the name of the subscribed message
     *
     * This function will return the name of the ROS message.
     *
     * **Returns**
     * {const std::string&} Returns the ROS message name.
     *
     */
    const std::string &ImageSubscribedMessage();
    const std::string &CameraInfoSubscribedMessage();
    /*
     * ### Obtaining the updating signal
     *
     * This function will return the address to a signal that is triggered every time
     * the data is updated.
     *
     * **Returns**
     * {boost::signal2::signal<void ()>*} Returns the address of the updating signal.
     *
     */
    boost::signals2::signal<void ()> *SignalUpdate();

  private:
    void ImageReader(const sensor_msgs::Image::ConstPtr &msg);
    void CameraInfoReader(const sensor_msgs::CameraInfo::ConstPtr &msg);

    ros::NodeHandle node_;
    ros::Subscriber image_input_, camera_input_;

    torero::CameraInfo camera_;
    torero::ImageFile image_;
    std::string image_message_name_, camera_message_name_;
    bool initialized_;
    std::size_t total_size_, pixel_byte_size_;

    boost::signals2::signal<void()> signal_update_;
  };
}

#endif // ROCINANTE_CAMERA_READER_H
