#ifndef KITTI_CONTROLLER_H
#define KITTI_CONTROLLER_H

#include "kitti/types.h"

#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <regex>
#include <string>
#include <vector>

namespace kitti {
  template<typename T>
  class KittiPointCloudReader;

  class KittiController
  {
  public:
    KittiController(const std::string &main_folder_path = "../KittiData/raw");

    const int go_to_dataset(const int dataset_number);
    const int move_dataset(const torero::Move position = torero::Move::Beginning);

    const int go_to_frame(const int frame_number);
    const int move_frame(const torero::Move position = torero::Move::Beginning);

    void set_module(KittiPointCloudReader<torero::PointXYZI> *cloud_reader);

  private:
    void look_for_folders();
    const bool set_dataset(const int id);
    const bool set_frame(const int id);

    kitti::UnknownElement cloud_reader_;

    int current_dataset_, current_frame_, total_datasets_, total_frames_;
    std::string main_folder_path_;
    bool error_;

    std::vector<kitti::DirectoryPath> subdirectory_paths_;
  };
}

#endif // KITTI_CONTROLLER_H
