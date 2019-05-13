#ifndef ROCINANTE_STREET_READER_H
#define ROCINANTE_STREET_READER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>

#include "rocinante/StreetMap.h"

#include "rocinante/rocinante_type_definitions.h"

#include <iostream>
#include <string>
#include <vector>
// signals and slots
#include <boost/bind.hpp>
#include <boost/signals2.hpp>

using namespace message_filters;

namespace rocinante {
  class ROSStreetReader
  {
  public:
    /*
     * ### Constructor
     *
     * This function will create the **ROS free space reader** *class*, you need to define
     * the ROS message name and the type of data input. The data `type` must coincide with
     * the `typename T`, see [[data types|ROS-free-space-reader#data-structure]] to see
     * which `type` you must select.
     *
     * **Arguments**
     * {const std::string&} message_name = ROS message name as listed at console using
     * `rostopic list` command.
     * {const FreeSpace} type = Type of input data.
     *
     * **Errors**
     * This will print a warning message in console if the `typename T` and `type`
     * do not match in the type of output.
     *
     */
    ROSStreetReader(const std::string &street_message_name,
                    const std::string &street_signal_message_name);

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
    void Resubscribe(const std::string &message_name, const rocinante::MessageType type);
    /*
     * ### Obtaining the address to the data vector
     *
     * This function will return the address to the vector containing all the read data.
     *
     * **Returns**
     * {const std::vector<T> *const} Returns the address of the vector containing the data.
     *
     */
    const std::vector<torero::Trajectory> *Streets() const;
//    const std::vector<torero::Street> *streets() const;
    /*
     * ### Obtaining the address to the data vector
     *
     * This function will return the address to the vector containing all the read data.
     *
     * **Returns**
     * {const std::vector<T> *const} Returns the address of the vector containing the data.
     *
     */
    const std::vector<torero::Signal> *StreetSignals() const;
    /*
     * ### Obtaining the name of the subscribed message
     *
     * This function will return the name of the ROS message.
     *
     * **Returns**
     * {const std::string&} Returns the ROS message name.
     *
     */
    const std::string &SubscribedMessage(const rocinante::MessageType type);
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
    void ReaderStreets(const rocinante::StreetMap::ConstPtr &msg);
//    void reader_signals(const rocinante::street_signals::ConstPtr &msg);

    ros::NodeHandle node_;
    ros::Subscriber street_input_, signal_input_;

    std::string street_message_name_, signal_message_name_;
//    std::vector<torero::Street> streets_;
    std::vector<torero::Trajectory> streets_;
    std::vector<torero::Signal> signals_;

    boost::signals2::signal<void()> signal_update_;
  };
}
#endif // ROCINANTE_STREET_READER_H
