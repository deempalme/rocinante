#include "rocinante/ros_laser_reader.h"

namespace rocinante {
  template<typename T>
  ROSLaserReader<T>::ROSLaserReader(const std::string message_name,
                                    const rocinante::LaserSensorType sensor_type) :
    node_(),
    type_(sensor_type),
    point_element_length_(0),
    point_step_(0),
    field_size_(0),
    field_type_(0),
    initialized_(false),
    message_name_(message_name),
    points_(0),
    laser_scan_parameters_()
  {
    Evaluate(points_);
  }

  template<typename T>
  void ROSLaserReader<T>::Resubscribe(const std::string message_name){
    initialized_ = false;
    input_.shutdown();
    message_name_ = message_name;
    Evaluate(points_);
  }

  template<typename T>
  const std::vector<T> *ROSLaserReader<T>::Points() const{
    return &points_;
  }

  template<typename T>
  const torero::SensorRI *ROSLaserReader<T>::LaserScanParameters() const{
    return &laser_scan_parameters_;
  }

  template<typename T>
  const std::string &ROSLaserReader<T>::SubscribedMessage(){
    return message_name_;
  }

  template<typename T>
  boost::signals2::signal<void ()> *ROSLaserReader<T>::SignalUpdate(){
    return &signal_update_;
  }

  // :::::::::::::::::::::::::::::::::::::::: PRIVATE ::::::::::::::::::::::::::::::::::::::::

  template<typename T>
  void ROSLaserReader<T>::DataReaderRI(const sensor_msgs::LaserScan::ConstPtr &msg){
    // Cleaning the previous segments
    points_.clear();
    // Points size
    std::size_t data_size{msg->ranges.size()};
    // Checking if there are intensity values
    const bool intensities{msg->intensities.size() == data_size};
    // Resizing the points vector to increase performance
    points_.resize(data_size);

    // Setting the Laser Scanner parameters
    laser_scan_parameters_.angle_min = msg->angle_min;
    laser_scan_parameters_.angle_max = msg->angle_max;
    laser_scan_parameters_.scan_time = msg->scan_time;
    laser_scan_parameters_.range_min = msg->range_min;
    laser_scan_parameters_.range_max = msg->range_max;
    laser_scan_parameters_.angle_increment = msg->angle_increment;
    laser_scan_parameters_.time_increment  = msg->time_increment;

    // Reading the data from ROS
    for(std::size_t i = 0u; i < data_size; ++i){
      // Range
      points_[i].data[0] = msg->ranges[i];
      // Intensity
      if(intensities) points_[i].data[1] = msg->intensities[i];
    }
    // Sending the updating signal
    signal_update_();
  }

  template<typename T>
  void ROSLaserReader<T>::DataReaderPC(const sensor_msgs::PointCloud2::ConstPtr &msg){
    // Cleaning the previous points
    points_.clear();
    // Point size
    const std::size_t data_size{msg->height * msg->width};
    const std::size_t row_step{static_cast<std::size_t>(msg->row_step)};
    const std::size_t point_step{static_cast<std::size_t>(msg->point_step)};
    const std::size_t total_rows{msg->data.size()};
    const std::size_t total_columns{row_step / point_step};
    // Checking if is first reading
    if(!initialized_){
      initialized_ = true;
      // Getting the bytes size of each point
      point_step_ = static_cast<std::size_t>(msg->point_step);
      // Number of fields (not counting sub-fields)
      const std::size_t pre_field_size{msg->fields.size()};
      // getting the real fields' size (counting sub-fields)
      for(std::size_t i = 0; i < pre_field_size; ++i){
        // Total number of sub-fields
        const uint32_t count = msg->fields[i].count;
        // Getting the byte size of the field (which is the same for each sub-field)
        const uint32_t data_type_size = GetByteSize(msg->fields[i].datatype);
        // Adding the number of sub-fields
        field_size_ += count;
        // Getting the properties for each sub-field
        for(uint32_t e = 0u; e < count; ++e){
          rocinante::PointField field;
          field.offset = static_cast<std::size_t>(msg->fields[i].offset + e * data_type_size);
          field.data_type = msg->fields[i].datatype;
          field.data_size = static_cast<std::size_t>(data_type_size);
          field_type_.push_back(field);
        }
      }
    }
    // Checking if point has the same number of elements than selected
    if(point_element_length_ != field_size_){
      Console(GetTypeString(point_element_length_), GetTypeString(field_size_));
      return;
    }
    // Resizing the points vector to increase performance
    points_.resize(data_size);

    // Reading the data from ROS
    for(std::size_t row = 0; row < total_rows; ++row){
      const std::size_t i{row * total_columns};
      const std::size_t current_row{row * row_step};
      // Traveling through points
      for(std::size_t column = 0; column < total_columns; ++column){
        const std::size_t e{i + column};
        // Current point's field
        const std::size_t current_point{current_row + column * point_step};
        // Traveling through point's fields
        for(std::size_t field = 0; field < field_size_; ++field){
          const std::size_t u{e + field};
          // Current byte position in binary data
          const std::size_t index{current_point + field_type_[field].offset};
          switch(field_type_[field].data_type){
            case sensor_msgs::PointField::INT8:
              points_[u].data[field] =
                  static_cast<float>(*(reinterpret_cast<const int8_t*>(msg->data.data() + index)));
            break;
            case sensor_msgs::PointField::UINT8:
              points_[u].data[field] =
                  static_cast<float>(*(msg->data.data() + index));
            break;
            case sensor_msgs::PointField::INT16:
              points_[u].data[field] =
                  static_cast<float>(*(reinterpret_cast<const int16_t*>(msg->data.data() + index)));
            break;
            case sensor_msgs::PointField::UINT16:
              points_[u].data[field] =
                  static_cast<float>(*(reinterpret_cast<const uint16_t*>(msg->data.data() + index)));
            break;
            case sensor_msgs::PointField::INT32:
              points_[u].data[field] =
                  static_cast<float>(*(reinterpret_cast<const int32_t*>(msg->data.data() + index)));
            break;
            case sensor_msgs::PointField::UINT32:
              points_[u].data[field] =
                  static_cast<float>(*(reinterpret_cast<const uint32_t*>(msg->data.data() + index)));
            break;
            case sensor_msgs::PointField::FLOAT32:
              points_[u].data[field] = *(reinterpret_cast<const float*>(msg->data.data() + index));
            break;
            case sensor_msgs::PointField::FLOAT64:
              points_[u].data[field] =
                  static_cast<float>(*(reinterpret_cast<const double*>(msg->data.data() + index)));
            break;
          }
        }
      }
    }
    // Sending the updating signal
    signal_update_();
  }

  // ––––––––––––––––––––––––––––––––– Evaluation –––––––––––––––––––––––––––––––––
  template<typename T>
  void ROSLaserReader<T>::Evaluate(const std::vector<torero::PointRI> &){
    point_element_length_ = 2;
    if(type_ == 0u)
      input_ = node_.subscribe(message_name_, 1, &ROSLaserReader<T>::DataReaderRI, this);
    else
      Console("torero::PointRI", "rocinante::LaserScan");
  }

  template<typename T>
  void ROSLaserReader<T>::Evaluate(const std::vector<torero::PointXY<float>> &){
    point_element_length_ = 2;
    if(type_ == 1u)
      input_ = node_.subscribe(message_name_, 1, &ROSLaserReader<T>::DataReaderPC, this);
    else
      Console("torero::PointXY<float>", "rocinante::PointCloud2");
  }

  template<typename T>
  void ROSLaserReader<T>::Evaluate(const std::vector<torero::PointXYZ> &){
    point_element_length_ = 3;
    if(type_ == 1u)
      input_ = node_.subscribe(message_name_, 1, &ROSLaserReader<T>::DataReaderPC, this);
    else
      Console("torero::PointXYZ", "rocinante::PointCloud2");
  }

  template<typename T>
  void ROSLaserReader<T>::Evaluate(const std::vector<torero::PointXYZI> &){
    point_element_length_ = 4;
    if(type_ == 1u)
      input_ = node_.subscribe(message_name_, 1, &ROSLaserReader<T>::DataReaderPC, this);
    else
      Console("torero::PointXYZI", "rocinante::PointCloud2");
  }

  template<typename T>
  void ROSLaserReader<T>::Evaluate(const std::vector<torero::PointXYZRGB> &){
    point_element_length_ = 6;
    if(type_ == 1u)
      input_ = node_.subscribe(message_name_, 1, &ROSLaserReader<T>::DataReaderPC, this);
    else
      Console("torero::PointXYZRGB", "rocinante::PointCloud2");
  }

  template<typename T>
  void ROSLaserReader<T>::Evaluate(const std::vector<torero::PointXYZRGBA> &){
    point_element_length_ = 7;
    if(type_ == 1u)
      input_ = node_.subscribe(message_name_, 1, &ROSLaserReader<T>::DataReaderPC, this);
    else
      Console("torero::PointXYZRGBA", "rocinante::PointCloud2");
  }

  template<typename T>
  const std::string ROSLaserReader<T>::GetTypeString(uint32_t input){
    std::string output;
    switch(input){
      case 2:
        output = "torero::pointXZ";
      break;
      case 3:
        output = "torero::PointXYZ";
      break;
      case 4:
        output = "torero::PointXYZI";
      break;
      case 6:
        output = "torero::PointXYZRGB";
      break;
      case 7:
        output = "torero::PointXYZRGBA";
      break;
      default:
        output = "Not recognized format";
      break;
    }
    return output;
  }

  template<typename T>
  const uint32_t ROSLaserReader<T>::GetByteSize(const uint data_type){
    if(data_type == sensor_msgs::PointField::INT8 || data_type == sensor_msgs::PointField::UINT8)
      return uint32_t{1u};
    else if(data_type == sensor_msgs::PointField::INT16
            || data_type == sensor_msgs::PointField::UINT16)
      return uint32_t{2u};
    else if(data_type == sensor_msgs::PointField::INT32
            || data_type == sensor_msgs::PointField::UINT32
            || data_type == sensor_msgs::PointField::FLOAT32)
      return uint32_t{4u};
    else if(data_type == sensor_msgs::PointField::FLOAT64)
      return uint32_t{8u};
  }

  template<typename T>
  void ROSLaserReader<T>::Console(const std::string &output, const std::string &selection){
    std::cout << "\n\033[1;41m Error: \033[0;1;38;5;174m \n"
              << "The sensor type you have selected does not match the output's type.\n"
              << "    Output: " << output << "  |  You must select: " << selection
              << "\033[0m" << std::endl;
  }

  template class ROSLaserReader<torero::PointRI>;
  template class ROSLaserReader<torero::PointXY<float>>;
  template class ROSLaserReader<torero::PointXYZ>;
  template class ROSLaserReader<torero::PointXYZI>;
  template class ROSLaserReader<torero::PointXYZRGB>;
  template class ROSLaserReader<torero::PointXYZRGBA>;
}
