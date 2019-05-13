#include "rocinante/ros_lane_reader.h"

namespace rocinante {
  ROSLaneReader::ROSLaneReader(const std::string &message_name) :
    node_(),
    input_(node_.subscribe(message_name, 1, &ROSLaneReader::DataReader, this)),
    message_name_(message_name),
    trajectories_(0)
  {}

  void ROSLaneReader::DataReader(const rocinante::Lanes::ConstPtr &msg){
    // Cleaning the previous trajectories
    trajectories_.clear();
    // Creating a trajectory's vertex
    torero::TrajectoryVertex vertex;
    // Total of left lanes
    std::size_t total_left_lanes{msg->left_lanes.size()};
    // Total of right lanes
    std::size_t total_right_lanes{msg->right_lanes.size()};
    // Resizing the trajectory vector to increase performance
    trajectories_.resize(total_left_lanes + total_right_lanes);
    // Trajectory counter
    std::size_t i{0u};

    // Reading LEFT LANES from ROS
    for(i; i < total_left_lanes; ++i){
      const rocinante::Lane &element{msg->left_lanes[i]};
      // Total number of vertices per lane
      std::size_t total_vertices{element.position.size()};
      // Resizing the trajectory vertices to increase performance
      trajectories_[i].resize(total_vertices);
      // Color
      Colorize(&element.line_type, &vertex.color.red, &vertex.color.green, &vertex.color.blue);
      // Line width
      vertex.line_width = torero::ToFloat(element.line_width);
      // Reading all the vertices
      for(std::size_t e = 0u; e < total_vertices; ++e){
        // Position
        vertex.position.x = element.position[e].x;
        vertex.position.y = element.position[e].y;
        vertex.position.z = element.position[e].z;
        trajectories_[i][e] = vertex;
      }
    }

    // Reading RIGHT LANES from ROS
    for(std::size_t u = 0u; u < total_right_lanes; ++u, ++i){
      const rocinante::Lane &element{msg->right_lanes[u]};
      // Total number of vertices per lane
      std::size_t total_vertices{element.position.size()};
      // Creating a trajectory
      trajectories_[i].resize(total_vertices);
      // Color
      Colorize(&element.line_type, &vertex.color.red, &vertex.color.green, &vertex.color.blue);
      // Line width
      vertex.line_width = element.line_width;
      // Reading all the vertices
      for(std::size_t e = 0u; e < total_vertices; ++e){
        // Position
        vertex.position.x = element.position[e].x;
        vertex.position.y = element.position[e].y;
        vertex.position.z = element.position[e].z;
        trajectories_[i][e] = vertex;
      }
    }
    signal_update_();
  }

  void ROSLaneReader::Resubscribe(const std::string &message_name){
    input_.shutdown();
    input_ = node_.subscribe(message_name, 1, &ROSLaneReader::DataReader, this);
    message_name_ = message_name;
  }

  const std::vector<torero::Trajectory> *ROSLaneReader::Trajectories() const{
    return &trajectories_;
  }

  const std::string &ROSLaneReader::SubscribedMessage(){
    return message_name_;
  }

  boost::signals2::signal<void ()> *ROSLaneReader::SignalUpdate(){
    return &signal_update_;
  }

  void ROSLaneReader::Colorize(const int *value, float *red, float *green, float *blue){
    switch(*value){
    case 1:
      *red = 99.0f;//255.0f;//99.0f;
      *green = 255.0f;//30.0f;//255.0f;
      *blue = 143.0f;//116.0f;//143.0f;
      break;
    case 2:
      *red = 255.0f;
      *green = 226.0f;
      *blue = 99.0f;
      break;
    case 3:
      *red = 99.0f;
      *green = 252.0f;
      *blue = 255.0f;
      break;
    default:
      *red = 99.0f;
      *green = 218.0f;
      *blue = 255.0f;
      break;
    }
  }
}
