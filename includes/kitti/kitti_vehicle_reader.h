#ifndef KITTI_VEHICLE_READER_H
#define KITTI_VEHICLE_READER_H

#include <boost/filesystem.hpp>
#include <boost/signals2.hpp>
#include <boost/bind.hpp>

#include <string>
#include <iostream>
#include <fstream>

namespace Visualizer {
#ifndef P_C_XYZ
#define P_C_XYZ
  union pointXYZ{
    struct{
      float x;
      float y;
      float z;
    };
    float data[3];
  };
#endif

#ifndef V_M_S
#define V_M_S
  union OrientationPYR{
    struct{
      float pitch;
      float yaw;
      float roll;
    };
    float data[3];
  };

  union OrientationXYZW{
    struct{
      float x;
      float y;
      float z;
      float w;
    };
    float data[4];
  };

  union CoordinatesLLA{
    struct{
      float latitude;
      float longitude;
      float altitude;
    };
    float data[3];
  };

  struct Vehicle{
    CoordinatesLLA position;
    pointXYZ position_xyz, velocity, acceleration;
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
}

namespace Kitti {
  class KittiVehicleReader
  {
  public:
    // This is to obtain Laser clouds information from KittiData,
    // use only the parent folder name where the velodyne_points folder is stored.
    // You must follow this folder structure:

    //  ∟– folder_path/   <- You define the folder_path when you create this class
    //      ∟– raw/
    //          ∟– 0001_sync/   <- Don't forget to delete the date in the folder's name
    //              ∟– velodyne_points/
    //                  ∟– data/
    //                  ∟– timestamps.txt
    //          ∟– 0002_sync/   <- Don't forget to delete the date in the folder's name
    //              ∟– velodyne_points/
    //                  ∟– data/
    //                  ∟– timestamps.txt
    //          ∟– 0003_sync/   <- Don't forget to delete the date in the folder's name
    //
    //             .
    //             .
    //             .

    // The folder path could be absolute or relative (to your executable folder)
    KittiVehicleReader(const std::string folder_path);
    ~KittiVehicleReader();

    // Returns the actual frame position
    const unsigned int actual_frame();
    // Returns the number of total frames in this folder
    const unsigned int total_frames();
    // Returns the address to the point cloud.
    // A vector with points using intensity and coordinates X, Y and Z
    const Visualizer::Vehicle *const vehicle();
    // Returns the timestamp when the point cloud was created.
    const std::string *const timestamp();

    const Visualizer::OrientationPYR *euler_angles() const;

    // Sets a new kitti Dataset,
    // returns false if the folder was not found
    // if you set datasetNumber = 1; then the complete folder name will be 0001_sync
    // Note that dates from the folder name were removed, example: "2011_09_26_0001_sync" --> "0001_sync"
    const bool set_dataset(const unsigned int dataset_number = 1);
    // Sets the frame in a specific frame number,
    // returns false if the frame number is bigger than existing frames
    // or if an error occurs (see application output to see messages)
    const bool goto_frame(const unsigned int frame_number = 0);
    // Connects th function goto_frame to an external boost::signal
    // It disconnects any old connection to goto_frame().
    void connect_frame(boost::signals2::signal<void (unsigned int)> *signal);
    // Connects th function set_dataset to an external boost::signal.
    // It disconnects any old connection to set_dataset().
    void connect_dataset(boost::signals2::signal<void (unsigned int)> *signal);
    // Connects th function read_next to an external boost::signal.
    // It disconnects any old connection to read_next().
    void connect_reader(boost::signals2::signal<void ()> *signal);
    // This signal is triggered after the frame's data is readed.
    boost::signals2::signal<void ()> *signal();

  private:
    Visualizer::Vehicle vehicle_;
    std::string timestamp_;
    std::string folder_path_;
    unsigned int frame_, total_frames_;

    boost::signals2::signal<void ()> signal_;
    boost::signals2::connection frame_connection_, dataset_connection_;
  };
}

#endif // KITTI_VEHICLE_READER_H
