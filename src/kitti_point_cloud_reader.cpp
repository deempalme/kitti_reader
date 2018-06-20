#include "kitti/kitti_point_cloud_reader.h"

namespace Kitti {
  template<typename T>
  KittiPointCloudReader<T>::KittiPointCloudReader(const std::string kitti_folder_path,
                                               const std::string subfolder_path) :
    point_cloud_(0),
    timestamp_(),
    folder_path_(kitti_folder_path),
    subfolder_path_(subfolder_path),
    frame_(0),
    total_frames_(0)
  {
    if(folder_path_.front() != '/') folder_path_ = "/" + folder_path_;
    if(folder_path_.back() != '/') folder_path_ += "/";

    if(subfolder_path_.front() != '/') subfolder_path_ = '/' + subfolder_path_;
    if(subfolder_path_.back() != '/') subfolder_path_ += '/';

    const std::string first_path(folder_path_);

    if(!boost::filesystem::exists(boost::filesystem::path(folder_path_))){
      folder_path_ = boost::filesystem::current_path().string() + folder_path_;

      if(!boost::filesystem::exists(boost::filesystem::path(folder_path_)))
        std::cout << "The file: " << first_path << " was not found.\n" <<
                     "  Neither: " << folder_path_ << "\n";
    }

    folder_path_ += "raw/0000_sync";
  }

  template<typename T>
  KittiPointCloudReader<T>::~KittiPointCloudReader(){
    if(frame_connection_.connected())
      frame_connection_.disconnect();
    if(dataset_connection_.connected())
      dataset_connection_.disconnect();
  }

  template<typename T>
  const bool KittiPointCloudReader<T>::SetDataset(const unsigned int dataset_number){
    const std::string dataset(std::to_string(dataset_number));
    folder_path_.replace(folder_path_.end() - 5 - dataset.size(), folder_path_.end() - 5, dataset);

    frame_ = 0;
    total_frames_ = 0;

    //opening the folder
    boost::filesystem::path directory(folder_path_ + subfolder_path_);

    if(boost::filesystem::exists(directory)){
      //viewing all the files inside the folder
      for(const boost::filesystem::directory_entry &entry :
          boost::filesystem::directory_iterator(directory))
        if(boost::filesystem::is_regular_file(entry.path()))
          total_frames_++;

      point_cloud_.clear();
      return true;
    }else
      std::cout << "\n\033[1;41m Error: \033[0;1;38;5;174m The folder: " << folder_path_
                << subfolder_path_ << " was not found. \033[0m\n" << std::endl;

    return false;
  }

  template<typename T>
  const bool KittiPointCloudReader<T>::GoToFrame(const unsigned int frame_number){
    bool result{false};
    std::size_t element_size{sizeof(T)};

    //check if the selected frame number is bigger than existing frames
    if(frame_number < total_frames_){

      //clear all the laser cloud information existing
      point_cloud_.clear();

      //this creates the file name string: if frame is 1 then the resulting name is
      //0000000001.bin
      frame_ = frame_number;
      std::string file_name("0000000000.bin");
      std::string number(std::to_string(frame_number));
      file_name.replace(file_name.begin() + 10 - number.size(), file_name.begin() + 10, number);

      //open the binary file of the velodyne_points folder
      std::ifstream file(std::string(folder_path_ + subfolder_path_ + file_name).c_str(),
                    std::ifstream::ate | std::ifstream::binary);
      if(file.is_open()){
        // this read the file size in bytes:
        std::ifstream::pos_type file_size = file.tellg();

        point_cloud_.resize(file_size / element_size);

        file.clear();
        file.seekg(0, file.beg);
        file.read((char*)point_cloud_.data(), file_size);

        //close the opened file
        file.close();
        result = true;

        signal_();
      }else
        //if an error occurs opening the file then this line will pop up
        std::cout << "the file: " << folder_path_ << subfolder_path_ << file_name
                  << " was not found." << std::endl;

      std::ifstream file_t(std::string(folder_path_ + subfolder_path_ + "timestamps.txt").c_str());
      if(file_t.is_open()){
        for(int i = 0; i <= frame_; i++){
          std::getline(file_t, timestamp_, '\n');
        }
        //close the open file
        file_t.close();
        //timestamp format : 0000-12-31 24:60:60.999
        timestamp_.resize(23);
      }else{
        result = false;
        //if an error occurs opening the timestamp file then this line will pop up
//        std::cout << "the timestamp file: " << folder_path_ << subfolder_path_
//                  << "timestamps.txt was not found." << std::endl;
      }
    }
    return result;
  }

  template<typename T>
  void KittiPointCloudReader<T>::ConnectFrame(boost::signals2::signal<void (unsigned int)> *signal){
    if(frame_connection_.connected())
      frame_connection_.disconnect();
    frame_connection_ = signal->connect(boost::bind(&KittiPointCloudReader<T>::GoToFrame, this, _1));
  }

  template<typename T>
  void KittiPointCloudReader<T>::ConnectDataset(boost::signals2::signal<void (unsigned int)> *signal){
    if(dataset_connection_.connected())
      dataset_connection_.disconnect();
    dataset_connection_ =
        signal->connect(boost::bind(&KittiPointCloudReader<T>::SetDataset, this, _1));
  }

  template<typename T>
  boost::signals2::signal<void ()> *KittiPointCloudReader<T>::Signal(){
    return &signal_;
  }

  template<typename T>
  const unsigned int KittiPointCloudReader<T>::ActualFrame(){
    return frame_;
  }

  template<typename T>
  const unsigned int KittiPointCloudReader<T>::TotalFrames(){
    return total_frames_;
  }

  template<typename T>
  const std::vector<T> *const KittiPointCloudReader<T>::PointCloud(){
    return &point_cloud_;
  }

  template<typename T>
  const std::string *const KittiPointCloudReader<T>::Timestamp(){
    return &timestamp_;
  }

  template class KittiPointCloudReader<Visualizer::PointXYZI>;
  template class KittiPointCloudReader<Visualizer::PointXYZRGB>;
  template class KittiPointCloudReader<Visualizer::PointXYZRGBI>;
  template class KittiPointCloudReader<Visualizer::PointXYZRGBA>;
}
