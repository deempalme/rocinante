#include "rocinante/ros_street_reader.h"

namespace rocinante {
  ROSStreetReader::ROSStreetReader(const std::string &street_message_name,
                                   const std::string &street_signal_message_name) :
    node_(),
    street_input_(node_.subscribe(street_message_name, 1,
                                  &ROSStreetReader::ReaderStreets, this)),
//    signal_input_(node_.subscribe(street_signal_message_name, 1,
//                                  &ROSStreetReader::reader_signals, this)),
    street_message_name_(street_message_name),
    signal_message_name_(street_signal_message_name),
    streets_(0),
    signals_(0)
  {}

  void ROSStreetReader::Resubscribe(const std::string &message_name, const MessageType type){
    if(type == rocinante::Streets){
      street_input_.shutdown();
      street_input_ = node_.subscribe(message_name, 1, &ROSStreetReader::ReaderStreets, this);
      street_message_name_ = message_name;
//    }else{
//      signal_input_.shutdown();
//      signal_input_ = node_.subscribe(message_name, 1, &ROSStreetReader::reader_signals, this);
//      signal_message_name_ = message_name;
    }
  }

  const std::vector<torero::Trajectory> *ROSStreetReader::Streets() const{
    return &streets_;
  }

  const std::vector<torero::Signal> *ROSStreetReader::StreetSignals() const{
    return &signals_;
  }

  const std::string &ROSStreetReader::SubscribedMessage(const MessageType type){
    if(type == rocinante::Streets)
      return street_message_name_;
//    else
    //      return signal_message_name_;
  }

  boost::signals2::signal<void ()> *ROSStreetReader::SignalUpdate(){
    return &signal_update_;
  }

  void ROSStreetReader::ReaderStreets(const rocinante::StreetMap::ConstPtr &msg){
    // Cleaning the previous data
    streets_.clear();
    // Total number of streets
    std::size_t total_streets{msg->streets.size()};
    // Resizing the streets vector to increase performance
    streets_.resize(total_streets);

    float u{0.0f};
    for(std::size_t i = 0u; i < total_streets; ++i, ++u){
      // Referencing the street message
      const rocinante::Street &element{msg->streets[i]};
      const float offset_z{u * 0.001f};
      // Referencing the street
      torero::Trajectory &street{streets_[i]};
//      torero::Street &street{streets_[i]};
      // Street properties:
      //   Left line
//      street.left = torero::StreetLineType(element.left_line);
      //   Rigth line
//      street.right = torero::StreetLineType(element.right_line);
      //   Sidewalk type
//      street.sidewalk = torero::StreetSidewalkType(element.sidewalk);
      //   Street name
//      street.name = element.name;
      // Total number of vertices
      std::size_t total_vertices{element.vertices.size()};
      // Obtaining the street's vertices data
      // Referencing the vector containing all the street's vertices
//      std::vector<torero::StreetVertex> &vertices{street.data};
      // Resizing the vertices vector to improve performance
      street.resize(total_vertices);
//      vertices.resize(total_vertices);
      // Going through all vertices
      for(std::size_t e = 0u; e < total_vertices; ++e){
        // Referencing the street's vertex message
        const rocinante::StreetPoint &vertex{element.vertices[e]};
        // Position
        street[e].position.x = static_cast<float>(vertex.position.x);
        street[e].position.y = static_cast<float>(vertex.position.y);
        street[e].position.z = static_cast<float>(vertex.position.z) + offset_z;
        // Color
        street[e].color.red = 94.2f;//157.0f;//99.0f;
        street[e].color.green = 102.6f;//171.0f;//255.0f;
        street[e].color.blue = 106.8f;//178.0f;//143.0f;
        // Dimension
        street[e].line_width = static_cast<float>(vertex.width);
        // Longitudinal angle (street inclination)
        street[e].angle = static_cast<float>(vertex.inclination);
      }
    }
    // Sending the updating signal
    signal_update_();
  }

//  void ROSStreetReader::reader_signals(const rocinante::street_signals::ConstPtr &msg){
//    // Cleaning the previous data
//    signals_.clear();
//    // Total number of signals
//    std::size_t total_signals{msg->signaling.size()};
//    // Resizing the signal vector to increase performance
//    signals_.resize(total_signals);

//    for(std::size_t i = 0u; i < total_signals; ++i){
//      // Referencing the signal message
//      const rocinante::street_signal &element{msg->signaling[i]};
//      // Referencing the signal
//      torero::Signal &signal{signals_[i]};
//      // Signal properties:
//      //   Position
//      signal.x = element.position_x;
//      signal.y = element.position_y;
//      signal.z = element.position_z;
//      //   Orientation
//      signal.pitch = element.pitch;
//      signal.yaw   = element.yaw;
//      signal.roll  = element.roll;
//      //   Visibility
//      signal.visible = element.visible;
//      //   Signal type
//      signal.type = torero::SignalType(element.type);
//    }
//  }
}
