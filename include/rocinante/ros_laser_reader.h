#ifndef ROCINANTE_LASER_READER_H
#define ROCINANTE_LASER_READER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include "rocinante/rocinante_type_definitions.h"

#include <iostream>
#include <string>
#include <vector>
// signals and slots
#include <boost/bind.hpp>
#include <boost/signals2.hpp>

using namespace message_filters;

namespace rocinante {
  template<typename T>
  class ROSLaserReader
  {
  public:
    /*
     * ### Constructor
     *
     * This function will create the **ROS laser reader** *class*, you need to define
     * the ROS message name and the type of data input. The data `type` must coincide with
     * the `typename T`, see [[data types|ROS-laser-reader#data-structure]] to see
     * which `type` you must select.
     *
     * **Arguments**
     * {const std::string} message_name = ROS message name as listed at console using
     * `rostopic list` command.
     * {const rocinante::LaserSensorType} sensor_type = Type of ROS message.
     *
     * **Errors**
     * This will print a warning message in console if the `typename T` and `type`
     * do not match in the type of output.
     *
     */
    ROSLaserReader(const std::string message_name,
                   const rocinante::LaserSensorType sensor_type = rocinante::LaserScan);

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
    void Resubscribe(const std::string message_name);
    /*
     * ### Obtaining the address to the data vector
     *
     * This function will return the address to the vector containing all the read data.
     *
     * **Returns**
     * {const std::vector<T> *const} Returns the address of the vector containing the data.
     *
     */
    const std::vector<T> *Points() const;
    /*
     * ### Obtaining the Laser Scanner parameters
     *
     * This function will return the address to the Laser Scanner parameters. Use this when
     * you have selected `torero::PointRI` as type. In any other type will return an empty
     * data structure.
     *
     * **Returns**
     * {const torero::SensorRI *const} Returns the address to the Laser Scanner parameters.
     *
     */
    const torero::SensorRI *LaserScanParameters() const;
    /*
     * ### Obtaining the name of the subscribed message
     *
     * This function will return the name of the ROS message.
     *
     * **Returns**
     * {const std::string&} Returns the ROS message name.
     *
     */
    const std::string &SubscribedMessage();
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
    void DataReaderRI(const sensor_msgs::LaserScan::ConstPtr &msg);
    void DataReaderPC(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void Evaluate(const std::vector<torero::PointRI> &);
    void Evaluate(const std::vector<torero::PointXY<float> > &);
    void Evaluate(const std::vector<torero::PointXYZ> &);
    void Evaluate(const std::vector<torero::PointXYZI> &);
    void Evaluate(const std::vector<torero::PointXYZRGB> &);
    void Evaluate(const std::vector<torero::PointXYZRGBA> &);

    inline const std::string GetTypeString(uint32_t input);
    inline const uint32_t GetByteSize(const uint data_type);

    void Console(const std::string &output, const std::string &selection);

    ros::NodeHandle node_;
    ros::Subscriber input_;

    LaserSensorType type_;
    std::size_t point_element_length_, point_step_, field_size_;
    std::vector<rocinante::PointField> field_type_;
    bool initialized_;

    std::string message_name_;
    std::vector<T> points_;
    torero::SensorRI laser_scan_parameters_;

    boost::signals2::signal<void()> signal_update_;
  };
}

#endif // ROCINANTE_LASER_READER_H
