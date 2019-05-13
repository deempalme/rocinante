#ifndef ROCINANTE_FREE_SPACE_READER_H
#define ROCINANTE_FREE_SPACE_READER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

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
  class ROSFreeSpaceReader
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
    ROSFreeSpaceReader(const std::string &message_name);

    /*
     * ### Obtaining the address to the Laser Scanner's parameters
     *
     * This function will return the address to the Laser Scanner's parameters.
     *
     * **Returns**
     * {const torero::SensorRI *const} Returns the address of the Laser Scanner's parameters.
     *
     */
    const torero::SensorRI *LaserScannerInfo() const;
    /*
     * ### Obtaining the address to the Occupancy Grid's parameters
     *
     * This function will return the address to the Occupancy Grid's parameters.
     *
     * **Returns**
     * {const torero::OccupancyGrid *const} Returns the address of the Occupancy
     * Grid's parameters.
     *
     */
    const torero::OccupancyGrid *OccupancyGridInfo() const;
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
     * {const std::vector<T> *const} Returns the address of the vector containing the data.
     *
     */
    const std::vector<T> *Segments() const;
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
    boost::signals2::signal<void ()> *SignalGridUpdate();
    boost::signals2::signal<void ()> *SignalUpdate();
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

  private:
    void Evaluate();

    void GetType(const std::vector<torero::GroundGrid>&);
    void GetType(const std::vector<torero::FreePolarGround2D>&);
    void GetType(const std::vector<torero::FreePolarGround3D>&);

    void ReaderOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void ReaderLaserScanner(const sensor_msgs::LaserScan::ConstPtr &msg);

    ros::NodeHandle node_;
    ros::Subscriber input_;

    FreeSpace return_type_;

    std::string message_name_;
    std::vector<T> segments_;

    torero::SensorRI laser_sensor_parameters_;
    torero::OccupancyGrid occupancy_grid_parameters_;

    boost::signals2::signal<void()> signal_update_, signal_grid_update_;
  };

  typedef ROSFreeSpaceReader<torero::GroundGrid> ROSFreeSpaceReaderGrid;
  typedef ROSFreeSpaceReader<torero::FreePolarGround2D> ROSFreeSpaceReaderLaser2D;
  typedef ROSFreeSpaceReader<torero::FreePolarGround3D> ROSFreeSpaceReaderLaser3D;
}

#endif // ROCINANTE_FREE_SPACE_READER_H
