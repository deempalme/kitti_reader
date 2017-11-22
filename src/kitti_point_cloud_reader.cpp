#include "includes/kitti_point_cloud_reader.h"

KittiPointCloudReader::KittiPointCloudReader(const char *folder_path) :
  folder_path_(folder_path),
  frame_(0),
  total_frames_(0)
{
  if(folder_path_.front() != '/') folder_path_ = "/" + folder_path_;
  if(folder_path_.back() != '/') folder_path_ += "/";

  std::string first_path(folder_path_);

  if(!boost::filesystem::exists(boost::filesystem::path(folder_path_))){
    folder_path_ = boost::filesystem::current_path().string() + folder_path_;

    if(!boost::filesystem::exists(boost::filesystem::path(folder_path_)))
      std::cout << "The file: " << first_path << " was not found.\n" <<
                   "  Neither: " << folder_path_ << "\n";
  }

  folder_path_ += "raw/0000_sync/velodyne_points/";
}

KittiPointCloudReader::~KittiPointCloudReader(){
  if(frame_connection_.connected())
    frame_connection_.disconnect();
  if(dataset_connection_.connected())
    dataset_connection_.disconnect();
  if(read_next_connection_.connected())
    read_next_connection_.disconnect();
}

const bool KittiPointCloudReader::set_dataset(const unsigned int dataset_number){
  bool result = false;

  std::string dataset(std::to_string(dataset_number));
  folder_path_.replace(folder_path_.end() - 22 - dataset.size(), folder_path_.end() - 22, dataset);

  frame_ = 0;
  total_frames_ = 0;

  //opening the folder
  boost::filesystem::path directory(folder_path_ + "data/");

  if(boost::filesystem::exists(directory)){
    //viewing all the files inside the folder
    dir_itr_ = boost::filesystem::directory_iterator(directory);
    for(const boost::filesystem::directory_entry &entry :
        boost::filesystem::directory_iterator(directory))
      if(boost::filesystem::is_regular_file(entry.path()))
        total_frames_++;

    result = true;
    point_cloud_.clear();
  }

  return result;
}

const bool KittiPointCloudReader::goto_frame(const unsigned int frame_number){
  bool result = false;
  size_t element_size = sizeof(Visualizer::pointXYZI);

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
    std::ifstream file(std::string(folder_path_ + "data/" + file_name).c_str(),
                  std::ifstream::ate | std::ifstream::binary);
    if(file.is_open()){
      // this read the file size in bytes:
      std::ifstream::pos_type file_size = file.tellg();

      point_cloud_.resize(file_size / element_size);

      file.seekg(0, file.beg);
      file.read((char*)point_cloud_.data(), file_size);

      //close the opened file
      file.close();
      result = true;

      signal_();
    }else
      //if an error occurs opening the file then this line will pop up
      std::cout << "the file: " << folder_path_ + file_name << " was not found." << std::endl;

    std::ifstream file_t(std::string(folder_path_ + "timestamps.txt").c_str());
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
      std::cout << "the timestamp file: " << folder_path_ << "timestamps.txt was not found."
                << std::endl;
    }
  }
  return result;
}

const bool KittiPointCloudReader::read_next(const bool start){
  bool result = false;

  if(start || frame_ >= total_frames_){
    frame_ = 0;
    dir_itr_ = boost::filesystem::directory_iterator(
          boost::filesystem::path(folder_path_ + "data/"));
  }

  point_cloud_.clear();

  if(boost::filesystem::is_regular_file(dir_itr_->path())){
    size_t element_size = sizeof(Visualizer::pointXYZIL);

    //open the binary file of the velodyne_points folder
    std::ifstream file(dir_itr_->path().string(), std::ifstream::ate | std::ifstream::binary);
    if(file.is_open()){
      // this read the file size in bytes:
      std::ifstream::pos_type file_size = file.tellg();

      size_t size{file_size / element_size};
      point_cloud_.resize(size);
      std::vector<Visualizer::pointXYZIL> temporal(size);

      file.seekg(0, file.beg);
      file.read((char*)temporal.data(), file_size);

      //close the opened file
      file.close();
      result = true;

      for(size_t i = 0; i < size; i++){
        unsigned short intensity{0};
        intensity |= temporal.at(i).i;
        point_cloud_[i].x = (temporal.at(i).x * 0.005f - 100.0f);
        point_cloud_[i].y = -(temporal.at(i).y * 0.005f - 100.0f);
        point_cloud_[i].z = -(temporal.at(i).z * 0.005f - 100.0f);
        point_cloud_[i].intensity = 1.0f - intensity / 255.0f;
      }

      signal_();
    }else
      //if an error occurs opening the file then this line will pop up
      std::cout << "the file: " << dir_itr_->path().string() << " was not found." << std::endl;
  }
  dir_itr_++;
  frame_++;

  return result;
}

boost::signals2::signal<void ()> *KittiPointCloudReader::signal(){
  return &signal_;
}

void KittiPointCloudReader::connect_frame(boost::signals2::signal<void (unsigned int)> *signal){
  if(frame_connection_.connected())
    frame_connection_.disconnect();
  frame_connection_ = signal->connect(boost::bind(&KittiPointCloudReader::goto_frame, this, _1));
}

void KittiPointCloudReader::connect_dataset(boost::signals2::signal<void (unsigned int)> *signal){
  if(dataset_connection_.connected())
    dataset_connection_.disconnect();
  dataset_connection_ = signal->connect(boost::bind(&KittiPointCloudReader::set_dataset, this, _1));
}

void KittiPointCloudReader::connect_reader(boost::signals2::signal<void ()> *signal){
  if(read_next_connection_.connected())
    read_next_connection_.disconnect();
  read_next_connection_ = signal->connect(boost::bind(&KittiPointCloudReader::read_next, this, false));
}

const unsigned int KittiPointCloudReader::actual_frame(){
  return frame_;
}

const unsigned int KittiPointCloudReader::total_frames(){
  return total_frames_;
}

const std::vector<Visualizer::pointXYZI> *const KittiPointCloudReader::point_cloud(){
  return &point_cloud_;
}

const std::string *const KittiPointCloudReader::timestamp(){
  return &timestamp_;
}
