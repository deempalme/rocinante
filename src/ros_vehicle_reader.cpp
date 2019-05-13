#include "rocinante/ros_vehicle_reader.h"

namespace rocinante {
  ROSVehicleReader::ROSVehicleReader(const std::string &odometry_message_name,
                                     const std::string &imu_message_name,
                                     const std::string &gps_message_name,
                                     const std::string &information_message_name) :
    node_(),
    input_odometry_(node_.subscribe(odometry_message_name, 1,
                                    &ROSVehicleReader::OdometryReader, this)),
    input_imu_(node_.subscribe(imu_message_name, 1, &ROSVehicleReader::IMUReader, this)),
    input_gps_(node_.subscribe(gps_message_name, 1, &ROSVehicleReader::GPSReader, this)),
    input_information_(node_.subscribe(information_message_name, 1,
                                       &ROSVehicleReader::InformationReader, this)),
    odometry_message_name_(odometry_message_name),
    imu_message_name_(imu_message_name),
    gps_message_name_(gps_message_name),
    information_message_name_(information_message_name),
    vehicle_{ 0.0f, 0.0f, 0.0f, // position
              0.0f, 0.0f, 0.0f, // position_xyz
              0.0f, 0.0f, 0.0f, // velocity
              0.0f, 0.0f, 0.0f, // acceleration
              0.0f, 0.0f, 0.0f, 0.0f, // orientation
              0.0f, 0.0f, 0.0f, // euler angles
              0.0f, 0.0f, 0.0f, // steering, speed, rpm
              0.0f, 0.0f, 0.0f, // fuel, gas, clutch
              0.0f, "P" },      // brake, gear
    total_read_(0u)
  {}

  void ROSVehicleReader::Resubscribe(const std::string &message_name,
                                     const SensorType sensor_type){
    switch(sensor_type){
      case SensorType::GPS:
        input_gps_.shutdown();
        input_gps_ = node_.subscribe(message_name, 1, &ROSVehicleReader::GPSReader, this);
        gps_message_name_ = message_name;
      break;
      case SensorType::IMU:
        input_imu_.shutdown();
        input_imu_ = node_.subscribe(message_name, 1, &ROSVehicleReader::IMUReader, this);
        imu_message_name_ = message_name;
      break;
      case SensorType::Odometry:
        input_odometry_.shutdown();
        input_odometry_ = node_.subscribe(message_name, 1, &ROSVehicleReader::OdometryReader, this);
        odometry_message_name_ = message_name;
      break;
      case SensorType::Information:
        input_information_.shutdown();
        input_information_ = node_.subscribe(message_name, 1, &ROSVehicleReader::InformationReader, this);
        information_message_name_ = message_name;
      break;
    }
  }

  const torero::CoordinatesLLA *ROSVehicleReader::Position() const{
    return &vehicle_.position;
  }

  const torero::PointXYZ *ROSVehicleReader::PositionXYZ() const{
    return &vehicle_.position_xyz;
  }

  const torero::PointXYZ *ROSVehicleReader::Velocity() const{
    return &vehicle_.velocity;
  }

  const torero::PointXYZ *ROSVehicleReader::Acceleration() const{
    return &vehicle_.acceleration;
  }

  const torero::OrientationXYZW *ROSVehicleReader::Orientation() const{
    return &vehicle_.orientation;
  }

  const torero::OrientationPYR *ROSVehicleReader::EulerAngles() const{
    return &vehicle_.euler;
  }

  const torero::Vehicle *ROSVehicleReader::Vehicle() const{
    return &vehicle_;
  }

  const float *ROSVehicleReader::SteeringAngle() const{
    return &vehicle_.steering_angle;
  }

  const float *ROSVehicleReader::RPM() const{
    return &vehicle_.rpm;
  }

  const std::string *ROSVehicleReader::Gear() const{
    return &vehicle_.gear;
  }

  const float *ROSVehicleReader::Fuel() const{
    return &vehicle_.fuel;
  }

  const float *ROSVehicleReader::Gas() const{
    return &vehicle_.gas;
  }

  const float *ROSVehicleReader::Clutch() const{
    return &vehicle_.clutch;
  }

  const float *ROSVehicleReader::Brake() const{
    return &vehicle_.brake;
  }

  const std::string &ROSVehicleReader::SubscribedMessage(const SensorType sensor_type){
    switch(sensor_type){
      case SensorType::GPS:
        return gps_message_name_;
      break;
      case SensorType::IMU:
        return imu_message_name_;
      break;
      case SensorType::Odometry:
        return odometry_message_name_;
      break;
      case SensorType::Information:
        return information_message_name_;
      break;
    }
  }

  boost::signals2::signal<void ()> *ROSVehicleReader::SignalUpdate(){
    return &signal_update_;
  }

  boost::signals2::signal<void (float, float)> *ROSVehicleReader::SignalSpeedometer(){
    return &signal_update_speedometer_;
  }

  boost::signals2::signal<void (float, float)> *ROSVehicleReader::SignalRotation(){
    return &signal_update_rotation_;
  }

  boost::signals2::signal<void (float, float, float, std::string)> *ROSVehicleReader::SignalPedals(){
    return &signal_update_pedals_;
  }

  void ROSVehicleReader::OdometryReader(const nav_msgs::Odometry::ConstPtr &msg){
    vehicle_.position_xyz.x = msg->pose.pose.position.x;
    vehicle_.position_xyz.y = msg->pose.pose.position.y;
    vehicle_.position_xyz.z = msg->pose.pose.position.z;

    vehicle_.orientation.x = torero::ToFloat(msg->pose.pose.orientation.x);
    vehicle_.orientation.y = torero::ToFloat(msg->pose.pose.orientation.y);
    vehicle_.orientation.z = torero::ToFloat(msg->pose.pose.orientation.z);
    vehicle_.orientation.w = torero::ToFloat(msg->pose.pose.orientation.w);

    vehicle_.velocity.x = torero::ToFloat(msg->twist.twist.linear.x);
    vehicle_.velocity.y = torero::ToFloat(msg->twist.twist.linear.y);
    vehicle_.velocity.z = torero::ToFloat(msg->twist.twist.linear.z);

    signaling(0x1);
  }

  void ROSVehicleReader::IMUReader(const sensor_msgs::Imu::ConstPtr &msg){
    vehicle_.acceleration.x = torero::ToFloat(msg->linear_acceleration.x);
    vehicle_.acceleration.y = torero::ToFloat(msg->linear_acceleration.y);
    vehicle_.acceleration.z = torero::ToFloat(msg->linear_acceleration.z);

    signaling(0x2);
  }

  void ROSVehicleReader::GPSReader(const sensor_msgs::NavSatFix::ConstPtr &msg){
    vehicle_.position.latitude  = msg->latitude;
    vehicle_.position.longitude = msg->longitude;
    vehicle_.position.altitude  = msg->altitude;

    signaling(0x4);
  }

  void ROSVehicleReader::InformationReader(const rocinante::Information::ConstPtr &msg){
    // Fuel percentage
    vehicle_.fuel = torero::ToFloat(msg->fuel_level);
    // Brake pressure level
    vehicle_.brake = torero::ToFloat(msg->brake);
    // Accelerator pressure level
    vehicle_.gas = torero::ToFloat(msg->accelerator);
    // Clutch pressure level
    vehicle_.clutch = torero::ToFloat(msg->clutch);
    // Gear position
    vehicle_.gear = msg->gear;
    // RPM
    vehicle_.rpm = torero::ToFloat(msg->rpm);

    // Orientation with euler angles
    vehicle_.euler.pitch = torero::ToFloat(msg->pitch);
    vehicle_.euler.yaw   = torero::ToFloat(msg->yaw);
    vehicle_.euler.roll  = torero::ToFloat(msg->roll);
    // Steering angle
    vehicle_.steering_angle = torero::ToFloat(msg->steering_angle);

    signaling(0x8);
    signal_update_speedometer_(torero::ToKMPH(msg->speed), vehicle_.rpm);
    signal_update_rotation_(vehicle_.euler.yaw, vehicle_.steering_angle);
    signal_update_pedals_(vehicle_.brake, vehicle_.gas, vehicle_.clutch, vehicle_.gear);
  }

  void ROSVehicleReader::signaling(const short control){
    if((total_read_ = total_read_ | control) == 0xF){
      total_read_ = 0;
      signal_update_();
    }
  }
}
