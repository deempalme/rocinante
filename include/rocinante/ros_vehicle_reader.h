#ifndef ROCINANTE_VEHICLE_READER_H
#define ROCINANTE_VEHICLE_READER_H

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/AccelWithCovariance.h>
#include "rocinante/Information.h"

#include "rocinante/rocinante_type_definitions.h"

#include <iostream>
#include <string>
#include <vector>
// signals and slots
#include <boost/bind.hpp>
#include <boost/signals2.hpp>

using namespace message_filters;

namespace rocinante {
  class ROSVehicleReader
  {
  public:
    ROSVehicleReader(const std::string &odometry_message_name,
                     const std::string &imu_message_name,
                     const std::string &gps_message_name,
                     const std::string &information_message_name);

    void Resubscribe(const std::string &message_name, const SensorType sensor_type);

    const torero::CoordinatesLLA *Position() const;
    const torero::PointXYZ *PositionXYZ() const;
    const torero::PointXYZ *Velocity() const;
    const torero::PointXYZ *Acceleration() const;
    const torero::OrientationXYZW *Orientation() const;
    const torero::OrientationPYR *EulerAngles() const;
    const torero::Vehicle *Vehicle() const;

    const float *SteeringAngle() const;

    const float *RPM() const;
    const std::string *Gear() const;
    const float *Fuel() const;

    const float *Gas() const;
    const float *Clutch() const;
    const float *Brake() const;
    /*
     * ### Obtaining the name of the subscribed message
     *
     * This function will return the name of the ROS message.
     *
     * **Returns**
     * {const std::string&} Returns the ROS message name.
     *
     */
    const std::string &SubscribedMessage(const SensorType sensor_type);
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
    boost::signals2::signal<void (float, float)> *SignalSpeedometer();
    boost::signals2::signal<void (float, float)> *SignalRotation();
    boost::signals2::signal<void (float, float, float, std::string)> *SignalPedals();

  private:
    void OdometryReader(const nav_msgs::Odometry::ConstPtr &msg);
    void IMUReader(const sensor_msgs::Imu::ConstPtr &msg);
    void GPSReader(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void InformationReader(const rocinante::Information::ConstPtr &msg);

    void signaling(const short control);

    ros::NodeHandle node_;
    ros::Subscriber input_odometry_, input_imu_, input_gps_, input_information_;

    std::string odometry_message_name_, imu_message_name_;
    std::string gps_message_name_, information_message_name_;
    torero::Vehicle vehicle_;
    unsigned int total_read_;

    boost::signals2::signal<void()> signal_update_;
    boost::signals2::signal<void(float, float)> signal_update_speedometer_, signal_update_rotation_;
    boost::signals2::signal<void(float, float, float, std::string)> signal_update_pedals_;
  };
}

#endif // ROCINANTE_VEHICLE_READER_H
