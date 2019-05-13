#ifndef ROCINANTE_OBJECT_READER_H
#define ROCINANTE_OBJECT_READER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Point.h>

#include "rocinante/Objects.h"

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
  class ROSObjectsReader
  {
  public:
    /*
     * ### Constructor
     *
     * This function will create the **ROS object reader** *class*, you only need to define
     * the ROS message name.
     *
     * **Arguments**
     * {const std::string&} message_name = ROS message name as listed at console using
     * `rostopic list` command.
     *
     */
    ROSObjectsReader(const std::string &message_name);

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
     * {const std::vector<torero::Object> *const} Returns the address of the vector
     * containing the data.
     *
     */
    const std::vector<T> *Objects() const;
    /*
     * ### Obtaining the address to the data vector containing the objects' path
     *
     * This function will return the address to the vector containing all the objects' path.
     *
     * **Returns**
     * {const std::vector<torero::Trajectory> *const} Returns the address of the vector
     * containing all the objects' path.
     *
     */
    const std::vector<torero::Trajectory> *ObjectsPath() const;
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
    void DataReader(const rocinante::Objects::ConstPtr &msg);
    void GetColor(float *red, float *green, float *blue, float speed);
    void FillPoints(const rocinante::Object &element, torero::Polygon *object);
    void FillPoints(const rocinante::Object &element, torero::Object *object);

    ros::NodeHandle node_;
    ros::Subscriber input_;

    std::string message_name_;
    std::vector<T> objects_;
    std::vector<torero::Trajectory> objects_path_;

    torero::PointXYZ color_palette_[4];

    boost::signals2::signal<void()> signal_update_;
  };

  typedef ROSObjectsReader<torero::Polygon> ROSPolygonReader;
  typedef ROSObjectsReader<torero::Object> ROSObjectReader;
}

#endif // ROCINANTE_OBJECT_READER_H
