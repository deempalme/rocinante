#ifndef ROCINANTE_ENUMERATORS_H
#define ROCINANTE_ENUMERATORS_H

namespace rocinante {
  enum class SensorType : unsigned int {
    Odometry    = 0u,
    IMU         = 1u,
    GPS         = 2u,
    Information = 3u
  };

  enum FreeSpace : unsigned int {
    // Choose this option if you are using nav_msgs::OccupancyGrid messages
    OccupancyGrid  = 0u,
    // Choose this one if you are using sensor_msgs::LaserScan messages WITHOUT height
    LaserScanner2D = 1u,
    // Choose this one if you are using sensor_msgs::LaserScan messages WITH height
    LaserScanner3D = 2u
  };

  enum LaserSensorType : unsigned int {
    // Choose this one whne using sensor_msgs::LaserScan ROS message
    LaserScan   = 0u,
    // Choose this one whne using sensor_msgs::PointCloud2 ROS message
    PointCloud2 = 1u
  };

  enum MessageType : unsigned int {
    // Choose this when you are using streets messages
    Streets = 0u,
    // Choose this when you are using signals messages
    Signals = 1u
  };
}

namespace torero {

#ifndef P_C_DENSE_E
#define P_C_DENSE_E
  enum LaserFieldType : unsigned int {
    tI    = 0u,
    tX    = 1u,
    tXY   = 2u,
    tXYZ  = 3u,
    tRGB  = 4u,
    tRGBA = 5u,
    tGRAY = 6u,
    tMONO = 7u
  };
#endif

#ifndef S_M_S_E
#define S_M_S_E
  enum class StreetLineType : unsigned int {
    // Solid lines:
    SingleSolidWhiteLine   = 0u,
    SingleSolidYellowLine  = 1u,
    DoubleSolidWhiteLine   = 2u,
    DoubleSolidYellowLine  = 3u,
    // Dashed lines:
    SingleDashedWhiteLine  = 4u,
    SingleDashedYellowLine = 5u,
    DoubleDashedWhiteLine  = 6u,
    DoubleDashedYellowLine = 7u,
    // No line
    NoLine                 = 8u
  };

  enum class StreetSidewalkType : unsigned int {
    RightSidewalk  = 0u,
    LeftSidewalk   = 1u,
    DoubleSidewalk = 2u,
    NoSidewalk     = 3u
  };

  enum class SignalType : unsigned int {
    Semaphore,
    SpeedLimit,
    Stop,
    TrafficCone
  };

#endif
}

#endif // ROCINANTE_ENUMERATORS_H
