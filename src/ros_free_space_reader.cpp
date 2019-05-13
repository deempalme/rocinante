#include "rocinante/ros_free_space_reader.h"

namespace rocinante {
  template<typename T>
  ROSFreeSpaceReader<T>::ROSFreeSpaceReader(const std::string &message_name) :
    node_(),
    message_name_(message_name),
    segments_(0),
    laser_sensor_parameters_(),
    occupancy_grid_parameters_()
  {
    GetType(segments_);
    Evaluate();
  }

  template<typename T>
  const torero::SensorRI *ROSFreeSpaceReader<T>::LaserScannerInfo() const{
    return &laser_sensor_parameters_;
  }

  template<typename T>
  const torero::OccupancyGrid *ROSFreeSpaceReader<T>::OccupancyGridInfo() const{
    return &occupancy_grid_parameters_;
  }

  template<typename T>
  void ROSFreeSpaceReader<T>::Resubscribe(const std::string &message_name){
    input_.shutdown();
    message_name_ = message_name;
    Evaluate();
  }

  template<typename T>
  const std::vector<T> *ROSFreeSpaceReader<T>::Segments() const{
    return &segments_;
  }

  template<typename T>
  boost::signals2::signal<void ()> *ROSFreeSpaceReader<T>::SignalGridUpdate(){
    return &signal_grid_update_;
  }

  template<typename T>
  boost::signals2::signal<void ()> *ROSFreeSpaceReader<T>::SignalUpdate(){
    return &signal_update_;
  }

  template<typename T>
  const std::string &ROSFreeSpaceReader<T>::SubscribedMessage(){
    return message_name_;
  }

  // :::::::::::::::::::::::::::::::::::::::: PRIVATE ::::::::::::::::::::::::::::::::::::::::

  template<typename T>
  void ROSFreeSpaceReader<T>::Evaluate(){
    switch(return_type_){
    case LaserScanner2D:
      input_ = node_.subscribe(message_name_, 1,
                               &ROSFreeSpaceReader<T>::ReaderLaserScanner, this);
    break;
    case LaserScanner3D:
      input_ = node_.subscribe(message_name_, 1,
                               &ROSFreeSpaceReader<T>::ReaderLaserScanner, this);
    break;
    default:
      input_ = node_.subscribe(message_name_, 1,
                               &ROSFreeSpaceReader<T>::ReaderLaserScanner, this);
    break;
    }
  }

  template<typename T>
  void ROSFreeSpaceReader<T>::GetType(const std::vector<torero::GroundGrid>&){
    return_type_ = OccupancyGrid;
  }

  template<typename T>
  void ROSFreeSpaceReader<T>::GetType(const std::vector<torero::FreePolarGround2D>&){
    return_type_ = LaserScanner2D;
  }

  template<typename T>
  void ROSFreeSpaceReader<T>::GetType(const std::vector<torero::FreePolarGround3D>&){
    return_type_ = LaserScanner3D;
  }

  template<typename T>
  void ROSFreeSpaceReader<T>::ReaderLaserScanner(const sensor_msgs::LaserScan::ConstPtr &msg){
    // Cleaning the previous segments
    segments_.clear();
    // Segments size
    std::size_t data_size{msg->ranges.size()};
    // Resizing the segments vector to increase performance
    segments_.resize(data_size);

    // Maximum distance range
    float distance_range{msg->range_max};
    // Total angle range of the sensor
    float angle_range{msg->angle_max - msg->angle_min};
    // Calculating the sensor's circunference at the maximum sensor distance range
    // Using this formula: C = 2 * radius * (arc_total_angle / 2)
    float circumference{distance_range * angle_range};
    // Maximum size of the segmentations
    float segment_max_size{circumference / static_cast<float>(data_size)};
    // Segment scale factor
    float scale{1.0f};

    // Reading the data from ROS
    for(std::size_t i = 0u; i < data_size; ++i){
      // Referencing a segment
      T &segment{segments_[i]};
      // Position
      segment.data[0] = msg->ranges[i];
      segment.data[1] = msg->angle_max - msg->angle_increment * static_cast<float>(i);
      // Color
      segment.data[3] = 0.0f;
      segment.data[4] = 80.0f;
      segment.data[5] = 200.0f;
      // Dimensions
      scale = segment_max_size * (segment.data[0] / distance_range);
      segment.data[6] = scale;
      segment.data[7] = scale;
      // Height
      if(return_type_ == LaserScanner3D)
        segment.data[8] = 3.0f * scale;
    }

    // Setting the Laser Scanner sensor's paramenters
    laser_sensor_parameters_.angle_min = msg->angle_min;
    laser_sensor_parameters_.angle_max = msg->angle_max;
    laser_sensor_parameters_.angle_increment = msg->angle_increment;
    laser_sensor_parameters_.time_increment = msg->time_increment;
    laser_sensor_parameters_.scan_time = msg->scan_time;
    laser_sensor_parameters_.range_min = msg->range_min;
    laser_sensor_parameters_.range_max = msg->range_max;

    signal_update_();
  }

  template<typename T>
  void ROSFreeSpaceReader<T>::ReaderOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    // Cleaning the previous segments
    segments_.clear();
    // Segments size
    std::size_t data_size{msg->data.size()};
    // Resizing the segments vector to increase performance
    segments_.resize(data_size);

    // Reading the data from ROS
    for(std::size_t i = 0u; i < data_size; ++i){
      // Probability
      segments_[i].data[0] = torero::ToFloat(msg->data[i]);
    }
    // Occupany Grid properties
    // Position
    occupancy_grid_parameters_.origin.x = torero::ToFloat(msg->info.origin.position.x);
    occupancy_grid_parameters_.origin.y = torero::ToFloat(msg->info.origin.position.y);
    occupancy_grid_parameters_.origin.z = torero::ToFloat(msg->info.origin.position.z);
    // Orientation
    occupancy_grid_parameters_.quaternion.x = torero::ToFloat(msg->info.origin.orientation.x);
    occupancy_grid_parameters_.quaternion.y = torero::ToFloat(msg->info.origin.orientation.y);
    occupancy_grid_parameters_.quaternion.z = torero::ToFloat(msg->info.origin.orientation.z);
    occupancy_grid_parameters_.quaternion.w = torero::ToFloat(msg->info.origin.orientation.w);
    // Dimensions
    occupancy_grid_parameters_.width = torero::ToFloat(msg->info.width) * msg->info.resolution;
    occupancy_grid_parameters_.length = torero::ToFloat(msg->info.height) * msg->info.resolution;
    // Cells
    occupancy_grid_parameters_.number_of_elements_through_width = msg->info.width;
    occupancy_grid_parameters_.number_of_elements_through_length = msg->info.height;

    signal_grid_update_();
  }

  template class ROSFreeSpaceReader<torero::GroundGrid>;
  template class ROSFreeSpaceReader<torero::FreePolarGround2D>;
  template class ROSFreeSpaceReader<torero::FreePolarGround3D>;
}
