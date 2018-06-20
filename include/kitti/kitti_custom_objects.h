#ifndef KITTI_CUSTOM_OBJECTS_READER_H
#define KITTI_CUSTOM_OBJECTS_READER_H

#include "kitti/types.h"
// Mathematical libraries
#include "algebraica/algebraica.h"
// Boost libraries
#include <boost/filesystem.hpp>
#include <boost/signals2.hpp>
#include <boost/bind.hpp>
// Standard libraries
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

namespace Kitti {
  class kittiCustomObjectsReader
  {
  public:
    /*
     * @File format:
     *
     * Lines starting with '#' are comments
     *
     * Each line has:
     *
     * @param float : width      [METERS],
     * @param float : length     [METERS],
     * @param float : height     [METERS],
     * @param float : position_x [METERS],
     * @param float : position_y [METERS],
     * @param float : position_z [METERS],
     * @param float : roll       [RADIANS],
     * @param float : pitch      [RADIANS],
     * @param float : yaw        [RADIANS],
     * @param float : red        [0.0 -> 255.0],
     * @param float : green      [0.0 -> 255.0],
     * @param float : blue       [0.0 -> 255.0],
     * @param float : line_width [METERS],
     * @param unsigned : is circle [1], is box [0]
     * @param unsigned : is in global_frame [1] is in vehicle_frame [0]
     * @param unsigned : has arrow [1]
     *
     */
    kittiCustomObjectsReader(const std::string kitti_folder_path,
                             const std::string subfolder_path);
    ~kittiCustomObjectsReader();

    const std::vector<Visualizer::Object> *const GetObstacles();
    const std::vector<Visualizer::Object> *const GetCylinders();
    const std::vector<Visualizer::Object> *const GetObstaclesFixed();
    const std::vector<Visualizer::Object> *const GetCylindersFixed();

    // Returns the actual frame position
    const unsigned int ActualFrame();
    // Returns the number of total frames in this folder
    const unsigned int TotalFrames();

    // Sets a new kitti Dataset,
    // returns false if the folder was not found
    // if you set datasetNumber = 1; then the complete folder name will be 0001_sync
    // Note that dates from the folder name were removed, example: "2011_09_26_0001_sync" --> "0001_sync"
    bool SetDataset(const unsigned int dataset_number = 1);
    // Sets the frame in a specific frame number,
    // returns false if the there is no objects in file
    bool GoToFrame(const unsigned int frame_number = 0);
    // Connects th function GoToFrame to an external boost::signal
    // It disconnects any old connection to GoToFrame().
    void ConnectFrame(boost::signals2::signal<void (unsigned int)> *signal);
    // Connects th function set_dataset to an external boost::signal.
    // It disconnects any old connection to set_dataset().
    void ConnectDataset(boost::signals2::signal<void (unsigned int)> *signal);
    // This signal is triggered after the frame's data is readed.
    boost::signals2::signal<void ()> *Signal();

  private:
    std::vector<Visualizer::Object> obstacles_;
    std::vector<Visualizer::Object> cylinders_;
    std::vector<Visualizer::Object> obstacles_fixed_;
    std::vector<Visualizer::Object> cylinders_fixed_;

    std::string folder_path_, subfolder_path_;
    unsigned int frame_, total_frames_, total_obstacles_;

    boost::signals2::signal<void ()> signal_;
    boost::signals2::connection frame_connection_, dataset_connection_;
  };
}

#endif // KITTI_CUSTOM_OBJECTS_READER_H
