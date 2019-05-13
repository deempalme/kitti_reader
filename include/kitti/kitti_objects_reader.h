#ifndef KITTI_OBJECTS_READER_H
#define KITTI_OBJECTS_READER_H

#include "kitti/types.h"

#include "algebraica/algebraica.h"

#include <boost/filesystem.hpp>
#include <boost/signals2.hpp>
#include <boost/bind.hpp>

#include <iostream>
#include <string>
#include <vector>

namespace Kitti {
  struct Obstacles{
    struct {
      float x;
      float y;
      float z;
    } dimension;
    std::vector<algebraica::vec3f> position;
    std::vector<algebraica::quaternionF> orientation;
    int first_frame = 0;
    int last_frame = 0;
    unsigned int type = 6;
  };

  class KittiObjectsReader
  {
  public:
    // This is to obtain Laser clouds information from KittiData,
    // use only the parent folder name where the velodyne_points folder is stored.
    // You must follow this folder structure:

    //  ∟– folder_path/   <- You define the folder_path when you create this class
    //      ∟– raw/
    //          ∟– 0001_sync/   <- Don't forget to delete the date in the folder's name
    //              ∟– tracklet_labels.xml
    //          ∟– 0002_sync/   <- Don't forget to delete the date in the folder's name
    //              ∟– tracklet_labels.xml
    //          ∟– 0003_sync/   <- Don't forget to delete the date in the folder's name
    //              ∟– tracklet_labels.xml
    //             .
    //             .
    //             .

    // The folder path could be absolute or relative (to your executable folder)
    KittiObjectsReader(const std::string folder_path);
    ~KittiObjectsReader();

    //returns the actual frame position
    unsigned int actual_frame() const;
    //returns the number of obstacles in the actual frame
    unsigned int total_obstacles() const;

    //this vector contains all the obstacles found in the actual frame
    const std::vector<torero::Object> *obstacles() const;

    //set a new kitti Dataset,
    //returns false if the folder was not found
    //if you set datasetNumber = 1; then the complete folder name will be 0001_sync
    //note that dates from the folder name were removed, example: "2011_09_26_0001_sync" --> "0001_sync"
    bool set_dataset(const unsigned int dataset_number = 1);
    //set the frame in a specific frame number,
    //returns false if there are no obstacles found
    bool go_to_frame(const unsigned int frame_number = 0);
    // Connects th function goto_frame to an external boost::signal
    // It disconnects any old connection to goto_frame().
    void connect_frame(boost::signals2::signal<void (unsigned int)> &signal);
    // Connects th function set_dataset to an external boost::signal.
    // It disconnects any old connection to set_dataset().
    void connect_dataset(boost::signals2::signal<void (unsigned int)> &signal);
    // This signal is triggered after the frame's data is readed.
    boost::signals2::signal<void ()> &signal();

  private:
    //this vector contains all the obstacles found in all frames
    std::vector<Kitti::Obstacles> all_obstacles_;
    std::vector<torero::Object> obstacles_;

    std::string folder_path_;
    unsigned int frame_, total_obstacles_;

    boost::signals2::signal<void ()> signal_;
    boost::signals2::connection frame_connection_, dataset_connection_;
  };
}

#endif // KITTI_OBJECTS_READER_H
