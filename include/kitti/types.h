#ifndef KITTI_TYPES_H
#define KITTI_TYPES_H

#include <string>

namespace torero {
#ifndef O_C_I
#define O_C_I
  union CoordinatesLLA{
    struct LLA{
      float latitude;
      float longitude;
      float altitude;
    } coordinates;
    float data[3] = { 0.0f, 0.0f, 0.0f };
  };

  union OrientationPYR{
    struct{
      float pitch;
      float yaw;
      float roll;
    } angles;
    float data[3] = { 0.0f, 0.0f, 0.0f };
  };

  union OrientationXYZW{
    struct{
      float x;
      float y;
      float z;
      float w;
    } axes;
    float data[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
  };
#endif

#ifndef P_C_XYZ
#define P_C_XYZ
  union PointXYZ{
    struct{
      float x;
      float y;
      float z;
    } point;
    float data[3] = { 0.0f, 0.0f, 0.0f };
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
    } point;
    float data[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
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
    } point;
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
    } point;
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
    } point;
    float data[7];
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

#ifndef V_M_S
#define V_M_S
  struct Vehicle{
    CoordinatesLLA position;
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


  // ------------------------------------------------------------------------------------ //
  // ------------------------------------ CONTROLLER ------------------------------------ //
  // ------------------------------------------------------------------------------------ //
#ifndef T_M_C
#define T_M_C
  enum class Move : int {
    Backward  = -1,
    Beginning = 0,
    Forward   = 1,
    Ending    = 2
  };
#endif
}

namespace kitti {
  enum class UnknownElementType : int {
    None      =-1,
    CloudI    = 0,
    CloudRGB  = 1,
    CloudRGBI = 2,
    CloudRGBA = 3
  };

  struct UnknownElement {
    UnknownElementType type = UnknownElementType::None;
    void *element           = nullptr;
  };

  struct DirectoryPath {
    int id = -1;
    std::string path;
  };
}

#endif // KITTI_TYPES_H
