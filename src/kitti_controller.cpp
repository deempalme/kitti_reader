#include "kitti/kitti_controller.h"
#include "kitti/kitti_point_cloud_reader.h"

namespace kitti {
  KittiController::KittiController(const std::string &main_folder_path) :
    cloud_reader_(),
    current_dataset_(0),
    current_frame_(0),
    total_datasets_(0),
    total_frames_(0),
    main_folder_path_(main_folder_path),
    error_(false),
    subdirectory_paths_(0)
  {
    if(main_folder_path_.front() != '/') main_folder_path_ = "/" + main_folder_path_;
    if(main_folder_path_.back() != '/') main_folder_path_ += "/";

    const std::string first_path(main_folder_path_);

    if(!boost::filesystem::exists(boost::filesystem::path(main_folder_path_))){
      main_folder_path_ = boost::filesystem::current_path().string() + main_folder_path_;

      if(!boost::filesystem::exists(boost::filesystem::path(main_folder_path_))){
        std::cout << "The folder: " << first_path << " was not found.\n"
                  << "++ Neither: " << main_folder_path_ << "\n" << std::endl;
        error_ = true;
      }
    }
    look_for_folders();
  }

  const int KittiController::go_to_dataset(const int dataset_number){
    if(error_) return -1;

    std::size_t id;
    const std::size_t total{subdirectory_paths_.size()};
    bool found{false};

    for(id = 0; id < total; ++id)
      if(dataset_number == subdirectory_paths_[id].id){
        found = true;
        break;
      }

    if(!found) return -1;

    if(!set_dataset(id)) return -1;

    return dataset_number;
  }

  const int KittiController::move_dataset(const torero::Move position){
    if(error_ || total_datasets_ <= 0) return -1;
    switch(position){
      case torero::Move::Backward:
        --current_dataset_;
      break;
      case torero::Move::Beginning:
        current_dataset_ = 0;
      break;
      case torero::Move::Ending:
        current_dataset_ = total_datasets_ - 1;
      break;
      default:
        ++current_dataset_;
      break;
    }
    if(current_dataset_ < 0)
      current_dataset_ = total_datasets_ - 1 - current_dataset_;
    else if(current_dataset_ >= total_datasets_)
      current_dataset_ -= total_datasets_ - 1;

    if(!set_dataset(current_dataset_)) return -1;

    return subdirectory_paths_[current_dataset_].id;
  }

  const int KittiController::go_to_frame(const int frame_number){
    if(error_ || frame_number < 0 || frame_number >= total_frames_) return -1;

    set_frame(frame_number);
    return current_frame_;
  }

  const int KittiController::move_frame(const torero::Move position){
    if(error_ || total_frames_ <= 0) return -1;
    switch(position){
      case torero::Move::Backward:
        --current_frame_;
      break;
      case torero::Move::Beginning:
        current_frame_ = 0;
      break;
      case torero::Move::Ending:
        current_frame_ = total_frames_ - 1;
      break;
      default:
        ++current_frame_;
      break;
    }
    if(current_frame_ < 0)
      current_frame_ = total_frames_ - 1 - current_frame_;
    else if(current_frame_ >= total_frames_)
      current_frame_ -= total_frames_ - 1;

    set_frame(current_frame_);

    return current_frame_;
  }

  void KittiController::set_module(KittiPointCloudReader<torero::PointXYZI> *cloud_reader){
    cloud_reader_ = kitti::UnknownElement{ kitti::UnknownElementType::CloudI,
                                           static_cast<void*>(cloud_reader) };
  }

  // :::::::::::::::::::::::::::::::::::::: PRIVATE ::::::::::::::::::::::::::::::::::::::

  void KittiController::look_for_folders(){
    subdirectory_paths_.clear();
    current_dataset_ = 0;
    total_datasets_ = 0;
    if(error_) return;

    subdirectory_paths_.reserve(32);
    boost::filesystem::path path(main_folder_path_);

    std::regex regex("[0-9]{4,10}_[a-z]", std::regex::ECMAScript | std::regex::icase);
    std::smatch result;
    std::string file_name;
    int path_id;

    if(boost::filesystem::exists(path)){
      boost::filesystem::directory_iterator end_itr;
      for(boost::filesystem::directory_iterator itr(path); itr != end_itr; ++itr)
        if(boost::filesystem::is_directory(itr->status())){
          std::regex_search(file_name = itr->path().filename().string(), result, regex);
          if(result.size() > 0){
            path_id = std::stoi(result[0].str().substr(0, result[0].str().size() - 2));
            subdirectory_paths_.push_back(kitti::DirectoryPath{ path_id, file_name });
          }
        }
    }
    subdirectory_paths_.shrink_to_fit();
    total_datasets_ = subdirectory_paths_.size();
  }

  const bool KittiController::set_dataset(const int id){
    boost::filesystem::path full_path(main_folder_path_ + subdirectory_paths_[id].path
                                      + "/velodyne_points/data");

    if(!boost::filesystem::exists(full_path))
      return false;

    current_dataset_ = id;
    total_frames_ =
        std::count_if(boost::filesystem::directory_iterator(full_path),
                      boost::filesystem::directory_iterator(),
                      static_cast<bool(*)(const boost::filesystem::path&)>(
                        boost::filesystem::is_regular_file));

    if(cloud_reader_.element != nullptr)
      switch(cloud_reader_.type){
        case kitti::UnknownElementType::CloudI:
          static_cast<kitti::KittiPointCloudReaderI*>(cloud_reader_.element)
              ->set_dataset(subdirectory_paths_[id].id);
        break;
      }

    return true;
  }

  const bool KittiController::set_frame(const int id){
    current_frame_ = id;

    if(cloud_reader_.element != nullptr)
      switch(cloud_reader_.type){
        case kitti::UnknownElementType::CloudI:
          static_cast<kitti::KittiPointCloudReaderI*>(cloud_reader_.element)->go_to_frame(id);
        break;
      }
  }
}
