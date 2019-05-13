#ifndef ROCINANTE_LANE_READER_H
#define ROCINANTE_LANE_READER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>

#include "rocinante/Lanes.h"

#include "rocinante/rocinante_type_definitions.h"

#include <iostream>
#include <string>
#include <vector>
// signals and slots
#include <boost/bind.hpp>
#include <boost/signals2.hpp>

using namespace message_filters;

namespace rocinante {
  class ROSLaneReader
  {
  public:
    /*
     * ### Constructor
     *
     * This function will create the **ROS lane reader** *class*, you only need to define
     * the ROS message name.
     *
     * **Arguments**
     * {const std::string&} message_name = ROS message name as listed at console using
     * `rostopic list` command.
     *
     */
    ROSLaneReader(const std::string &message_name);

    /*
     * ### Changing the ROS topic (message)
     *
     * This will change the read message, the new message must be the same type than the previous.
     *
     * **Arguments**
     * {const std::string&} message_name = ROS message name as listed at console using
     * `rostopic list` command.
     *
     */
    void Resubscribe(const std::string &message_name);
    /*
     * ### Obtaining the address to the data vector
     *
     * This function will return the address to the vector containing all the read data.
     *
     * **Returns**
     * {const std::vector<torero::Trajectory> *const} Returns the address of the vector
     * containing the data.
     *
     */
    const std::vector<torero::Trajectory> *Trajectories() const;
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
    void DataReader(const rocinante::Lanes::ConstPtr &msg);

    void Colorize(const int *value, float *red, float *green, float *blue);

    ros::NodeHandle node_;
    ros::Subscriber input_;

    std::string message_name_;
    std::vector<torero::Trajectory> trajectories_;

    boost::signals2::signal<void()> signal_update_;
  };
}

#endif // ROCINANTE_LANE_READER_H
