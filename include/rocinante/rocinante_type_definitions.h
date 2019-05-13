#ifndef ROCINANTE_TYPE_DEFINITIONS_H
#define ROCINANTE_TYPE_DEFINITIONS_H

#include "rocinante/rocinante_constant_expressions.h"
#include "rocinante/rocinante_enumerators.h"

#include <stdint.h>
#include <string>
#include <vector>

namespace rocinante {
  struct PointField {
    std::size_t offset;
    uint8_t data_type;
    std::size_t data_size;
  };
}

namespace torero {
#ifndef P_C_XY_LL
#define P_C_XY_LL
  template<typename T>
  union PointXY{
    struct{
      T x;
      T y;
    };
    T data[2];
  };

  template<typename T>
  union pointLL{
    struct{
      T latitude;
      T longitude;
    };
    T data[2];
  };
#endif

#ifndef P_C_XYZ
#define P_C_XYZ
  union PointXYZ{
    struct{
      float x;
      float y;
      float z;
    };
    float data[3];
  };
#endif

#ifndef P_C_XYZI
#define P_C_XYZI
  union PointXYZI{
    struct{
      float x;
      float y;
      float z;
      float intensity;
    };
    float data[4];
  };
#endif

#ifndef P_C_XYZRGB
#define P_C_XYZRGB
  union PointXYZRGB{
    struct{
      float x;
      float y;
      float z;
      float r;
      float g;
      float b;
    };
    float data[6];
  };

  union PointXYZRGBI{
    struct{
      float x;
      float y;
      float z;
      float r;
      float g;
      float b;
      float intensity;
    };
    float data[7];
  };

  union PointXYZRGBA{
    struct{
      float x;
      float y;
      float z;
      float r;
      float g;
      float b;
      float a;
    };
    float data[7];
  };
#endif

#ifndef P_C_RI
#define P_C_RI
  union PointRI{
    struct{
      // Range data [m] (Note: values < range_min or > range_max should be discarded)
      float range;
      // Intensity data [device-specific units].  If your device does not provide
      // intensities, please do not set the value (it should be 1.0f as default).
      float intensity;
    };
    float data[2] = { 0.0f, 1.0f };
  };

  union SensorRI{
    struct{
      // Start angle of the scan [rad]
      float angle_min;
      // End angle of the scan [rad]
      float angle_max;
      // Angular distance between measurements [rad]
      float angle_increment;

      // Time between measurements [seconds] - if your scanner is moving,
      // this will be used in interpolating position of 3d points
      float time_increment;

      // Time between scans [seconds]
      float scan_time;

      // Minimum range value [m]
      float range_min;
      // Maximum range value [m]
      float range_max;
    };
    float data[9] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, };
  };
#endif

#ifndef O_C_I
#define O_C_I
  union CoordinatesLLA{
    struct{
      float latitude;
      float longitude;
      float altitude;
    };
    float data[3];
  };

  union OrientationPYR{
    struct{
      float pitch;
      float yaw;
      float roll;
    };
    float data[3] = { 0.0f, 0.0f, 0.0f };
  };

  union OrientationXYZW{
    struct{
      float x;
      float y;
      float z;
      float w;
    };
    float data[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
  };
#endif

#ifndef C_RGB_A
#define C_RGB_A
  union ColorRGB{
    struct{
      float red;
      float green;
      float blue;
    };
    float data[3];
  };

  union ColorRGBA{
    struct{
      float red;
      float green;
      float blue;
      float alpha;
    };
    float data[4];
  };
#endif

#ifndef P_C_DENSE_T
#define P_C_DENSE_T
  struct PointFields{
    LaserFieldType type;
    unsigned int offset;
    unsigned short byte_size;
  };

  struct DensePoints{
    unsigned int stamp;
    unsigned int width;
    std::vector<PointFields> fields;
    bool is_bigendian = false;
    unsigned int point_step;
    unsigned char *data = nullptr;
  };
#endif

#ifndef V_M_S
#define V_M_S
  struct Vehicle{
    CoordinatesLLA position = CoordinatesLLA{ 0.0f, 0.0f, 0.0f };
    PointXYZ position_xyz, velocity, acceleration;
    OrientationXYZW orientation;
    OrientationPYR euler;

    float steering_angle;

    float speed;
    float rpm;
    float fuel;

    float gas;
    float clutch;
    float brake;

    std::string gear;
  };
#endif

#ifndef O_M_A
#define O_M_A
  struct Arrow{
    // Arrow's orientation [quaternion]
    OrientationXYZW orientation;
    // Arrow's length in meters
    float length = 1.0f;
    // Display the arrow
    bool display = true;
  };
#endif

#ifndef O_M_D
#define O_M_D
  struct Object{
    // Object position (LOCATED at the object's center)
    PointXYZ position;
    // Object orientation (in radians)
    OrientationXYZW orientation;
    // Object's RGBA color
    ColorRGBA color = ColorRGBA{ 255.0f, 255.0f, 255.0f, 255.0f };
    // Arrow's properties:
    Arrow arrow;
    // Object's size in meters
    float width  = 1.0f;
    float length = 1.0f;
    float height = 1.0f;
    // Displays the object as a solid (filled faces)
    bool solid = false;
    // Line width in meters
    float line_width = 0.1f;
    std::string name;
  };
#endif

#ifndef O_M_P
#define O_M_P
  struct Polygon{
    // Object's center position
    PointXYZ position;
    // Object polygon shape, the origin is located at **position**
    std::vector<PointXYZ> points;
    // Object orientation (in radians)
    OrientationXYZW orientation;
    // Object's RGBA color
    ColorRGBA color = ColorRGBA{ 255.0f, 255.0f, 255.0f, 255.0f };
    // Arrow's properties:
    Arrow arrow;
    // Object's height in meters
    float height = 1.0f;
    // Displays the object as a solid (filled faces)
    bool solid = false;
    std::string name;
  };
#endif

#ifndef G_M_E
#define G_M_E
  union GroundGrid{
    struct{
      // Probability if occupied (-1.0f if unknown)
      float probability;
      // Box height
      float height;
    };
    float data[2] = {-1.0f, 0.0f};
  };

  union Ground2D{
    struct{
      // RGBA color (range 0.0f to 255.0f)
      ColorRGBA color;
    };
    float data[4] = { 255.0f, 255.0f, 255.0f, 255.0f };
  };

  union Ground3D{
    struct{
      // RGBA color (range 0.0f to 255.0f)
      ColorRGBA color;
      // cube's Height in meters
      float height;
    };
    float data[5] = { 255.0f, 255.0f, 255.0f, 255.0f, 0.0f };
  };

  union FreeGround2D{
    struct{
      // Position
      PointXYZ position;
      // RGBA color (range 0.0f to 255.0f)
      ColorRGBA color;
      // Dimensions
      float width;
      float length;
    };
    float data[9] = { 0.0f, 0.0f, 0.0f, 255.0f, 255.0f, 255.0f, 255.0f, 1.0f, 1.0f };
  };

  union FreeGround3D{
    struct{
      // Position
      PointXYZ position;
      // RGBA color (range 0.0f to 255.0f)
      ColorRGBA color;
      // Dimensions
      float width;
      float length;
      float height;
    };
    float data[10] = { 0.0f, 0.0f, 0.0f, 255.0f, 255.0f, 255.0f, 255.0f, 1.0f, 1.0f, 1.0f };
  };

  union FreePolarGround2D{
    struct{
      // Distance
      float distance;
      // Angle
      float angle;
      // RGBA color (range 0.0f to 255.0f)
      ColorRGBA color;
      // Dimensions
      float width;
      float length;
    };
    float data[8] = { 0.0f, 0.0f, 255.0f, 255.0f, 255.0f, 255.0f, 1.0f, 1.0f };
  };

  union FreePolarGround3D{
    struct{
      // Distance
      float distance;
      // Angle
      float angle;
      // RGBA color (range 0.0f to 255.0f)
      ColorRGBA color;
      // Dimensions
      float width;
      float length;
      float height;
    };
    float data[9] = { 0.0f, 0.0f, 255.0f, 255.0f, 255.0f, 255.0f, 1.0f, 1.0f, 1.0f };
  };

  struct OccupancyGrid {
    // The map width (in Y axis) [meters]
    float width  = 100.0f;
    // The map length (in X axis) [meters]
    float length = 100.0f;
    // Number of cells through Y axis
    unsigned int number_of_elements_through_width  = 100u;
    // Number of cells through X axis
    unsigned int number_of_elements_through_length = 100u;
    // Map origin
    PointXYZ origin;
    // Map orientation
    OrientationXYZW quaternion;
  };
#endif

#ifndef T_M_E
#define T_M_E
  struct TrajectoryVertex{
    // Object position
    PointXYZ position;
    // Object's color (0 to 255)
    ColorRGBA color = ColorRGBA{ 255.0f, 255.0f, 255.0f, 255.0f };
    // Line width in meters
    float line_width = 1.0f;
    // Rotation angle in radians
    // This rotation affects only the longitudinal axis
    // (imaginary line from this vertex to the next one)
    float angle = 0.0f;
  };

  typedef std::vector<TrajectoryVertex> Trajectory;
#endif

#ifndef S_M_S_T
#define S_M_S_T
  struct StreetVertex {
    // Position
    union {
      PointXYZ position;
      CoordinatesLLA coordinates = CoordinatesLLA{ 0.0f, 0.0f, 0.0f };
    };
    // Dimension
    float width = 1.0f;
    // Rotation angle in radians
    // This rotation affects only the longitudinal axis
    // (imaginary line from this vertex to the next one)
    // (Street's inclination)
    float angle = 0.0f;
  };

  struct Street {
    // Street lines
    StreetLineType left  = StreetLineType::NoLine;
    StreetLineType right = StreetLineType::NoLine;
    // Sidewalks
    StreetSidewalkType sidewalk = StreetSidewalkType::NoSidewalk;
    // Street name
    std::string name;
    // vertices
    std::vector<StreetVertex> data;
  };

  struct Signal {
    // Position
    union {
      PointXYZ position;
      CoordinatesLLA coordinates = CoordinatesLLA{ 0.0f, 0.0f, 0.0f };
    };
    // Orientation (Euler angles in radians)
    OrientationPYR euler;
    // Type of signal
    SignalType type = SignalType::SpeedLimit;
    // Visibility
    bool visible = true;
  };
#endif

#ifndef V_C_M_T
#define V_C_M_T
  struct ImageFile{
    // Use this in case each data's component is 16 bits long
    bool is_16bits      = false;
    // Use this in case the colors are inverted. Example: BGR instead og RGB
    bool is_inverted    = false;
    int width           = 0;
    int height          = 0;
    int components_size = 0;
    unsigned char *data = nullptr;
  };

  struct CameraInfo {
    int height;
    int width;
  };
#endif
}

#endif // ROCINANTE_TYPE_DEFINITIONS_H
