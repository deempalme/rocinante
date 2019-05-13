#include "rocinante/ros_objects_reader.h"

namespace rocinante {
  template<typename T>
  ROSObjectsReader<T>::ROSObjectsReader(const std::string &message_name) :
    node_(),
    input_(node_.subscribe(message_name, 1, &ROSObjectsReader<T>::DataReader, this)),
    message_name_(message_name),
    objects_(0),
    objects_path_(0),
    color_palette_{{51.0f, 127.5f, 178.5f}, {0.0f, 255.0f, 0.0f},
                   {255.0f, 255.0f, 0.0f}, {255.0f, 0.0f, 0.0f}}
  {}

  template<typename T>
  void ROSObjectsReader<T>::DataReader(const rocinante::Objects::ConstPtr &msg){
    // Cleaning the previous objects
    objects_.clear();
    objects_path_.clear();
    // Vector size
    std::size_t data_size{msg->objects.size()};
    // Resizing the objects vector to increase performance
    objects_.resize(data_size);
    objects_path_.resize(data_size);
    // Element index for objects' paths
    std::size_t e{0u};

    // Reading the data from ROS
    for(std::size_t i = 0u; i < data_size; ++i){
      const rocinante::Object &element{msg->objects[i]};
      // Creating a reference to object
      T &object{objects_[i]};
      // Filling the polygon points if is a torero::Polygon
      FillPoints(element, &object);
      // Position
      object.position.x = torero::ToFloat(element.pose.pose.position.x);
      object.position.y = torero::ToFloat(element.pose.pose.position.y);
      object.position.z = torero::ToFloat(element.pose.pose.position.z);
      // Orientation (quaternion)
      object.orientation.x = object.arrow.orientation.x = torero::ToFloat(element.pose.pose.orientation.x);
      object.orientation.y = object.arrow.orientation.y = torero::ToFloat(element.pose.pose.orientation.y);
      object.orientation.z = object.arrow.orientation.z = torero::ToFloat(element.pose.pose.orientation.z);
      object.orientation.w = object.arrow.orientation.w = torero::ToFloat(element.pose.pose.orientation.w);
      // Height
      object.height = torero::ToFloat(element.height);
      // Name
      object.name = element.name;
      // Color
      switch(element.classification){
      case 0:
        object.color.red   = 255.0f;
        object.color.green = 107.0f;
        object.color.blue  = 99.0f;
        break;
      case 1:
        object.color.red   = 255.0f;
        object.color.green = 208.0f;
        object.color.blue  = 99.0f;
        break;
      case 2:
        object.color.red   = 208.0f;
        object.color.green = 255.0f;
        object.color.blue  = 99.0f;
        break;
      case 3:
        object.color.red   = 99.0f;
        object.color.green = 255.0f;
        object.color.blue  = 255.0f;
        break;
      case 4:
        object.color.red   = 99.0f;
        object.color.green = 169.0f;
        object.color.blue  = 255.0f;
        break;
      default:
        object.color.red   = 226.0f;
        object.color.green = 247.0f;
        object.color.blue  = 255.0f;
        break;
      }
      const float speed(std::sqrt(element.velocity.twist.linear.x * element.velocity.twist.linear.x +
                                  element.velocity.twist.linear.y * element.velocity.twist.linear.y +
                                  element.velocity.twist.linear.z * element.velocity.twist.linear.z));

      // PATHS ----------------------------------------------
      // Size of path
      const std::size_t path_size{element.trajectory.size()};
      const float trajectory_width{element.trajectory_width};

      if(path_size > 0u){
        // Creating a reference to a Visualizer trajectory vertex
        torero::Trajectory *path{&objects_path_[e]};
        // Resizing trajectory vector to improve performance
        path->resize(path_size);
        // Going through all vertices
        for(std::size_t u = 0u; u < path_size; ++u){
          // Creating a reference to one ROS trajectory vertex
          const auto &point{element.trajectory[u]};
          // Position
          (*path)[u].position.x = torero::ToFloat(point.x);
          (*path)[u].position.y = torero::ToFloat(point.y);
          (*path)[u].position.z = torero::ToFloat(point.z);
          // Color
          GetColor(&(*path)[u].color.red, &(*path)[u].color.green, &(*path)[u].color.blue, speed);
          // Width
          (*path)[u].line_width = trajectory_width;
        }
        ++e;
      }
    }
    // Eliminating unfilled paths
    objects_path_.resize(e);
    // triggering the signal to update the data on external libraries
    signal_update_();
  }

  template<typename T>
  void ROSObjectsReader<T>::GetColor(float *red, float *green, float *blue, float speed){
    speed /= 50.0f;

    int idx1 = 0;// Our desired color will be between these two indexes in "color".
    int idx2 = 0;
    float fractBetween = 0.0f;// Fraction between "idx1" and "idx2" where our value is.

    if(speed <= 0.0f)
      idx1 = idx2 = 0;
    else if(speed >= 1.0f)
      idx1 = idx2 = 3;
    else{
      speed = speed * 3.0f;
      idx1 = static_cast<int>(std::floor(speed));// Our desired color will be after this index.
      idx2 = idx1 + 1;// ... and before this index (inclusive).
      fractBetween = speed - static_cast<float>(idx1);// Distance between the two indexes (0-1).
    }

    *red   = (color_palette_[idx2].x - color_palette_[idx1].x)
             * fractBetween + color_palette_[idx1].x;
    *green = (color_palette_[idx2].y - color_palette_[idx1].y)
             * fractBetween + color_palette_[idx1].y;
    *blue  = (color_palette_[idx2].z - color_palette_[idx1].z)
             * fractBetween + color_palette_[idx1].z;
  }

  template<typename T>
  void ROSObjectsReader<T>::FillPoints(const rocinante::Object &element,
                                       torero::Polygon *object){
    const std::size_t total_vertices{element.footprint.points.size()};
    object->points.resize(total_vertices);
    for(std::size_t e = 0; e < total_vertices; ++e){
      object->points[e].x = element.footprint.points[e].x;
      object->points[e].y = element.footprint.points[e].y;
      object->points[e].z = element.footprint.points[e].z;
    }
  }

  template<typename T>
  void ROSObjectsReader<T>::FillPoints(const rocinante::Object &element,
                                       torero::Object *object){
    if(element.footprint.points.size() > 0){
      object->width  = element.footprint.points[0].y;
      object->length = object->arrow.length = element.footprint.points[0].x;
      object->line_width = 0.15f;
    }
  }

  template<typename T>
  void ROSObjectsReader<T>::Resubscribe(const std::string &message_name){
    input_.shutdown();
    input_ = node_.subscribe(message_name, 1, &ROSObjectsReader<T>::DataReader, this);
    message_name_ = message_name;
  }

  template<typename T>
  const std::vector<T> *ROSObjectsReader<T>::Objects() const{
    return &objects_;
  }

  template<typename T>
  const std::vector<torero::Trajectory> *ROSObjectsReader<T>::ObjectsPath() const{
    return &objects_path_;
  }

  template<typename T>
  const std::string &ROSObjectsReader<T>::SubscribedMessage(){
    return message_name_;
  }

  template<typename T>
  boost::signals2::signal<void ()> *ROSObjectsReader<T>::SignalUpdate(){
    return &signal_update_;
  }

  template class ROSObjectsReader<torero::Polygon>;
  template class ROSObjectsReader<torero::Object>;
}
