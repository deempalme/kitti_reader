#include "kitti/kitti_custom_objects.h"

namespace Kitti {
  kittiCustomObjectsReader::kittiCustomObjectsReader(const std::string kitti_folder_path,
                                                     const std::string subfolder_path) :
    obstacles_(0),
    cylinders_(0),
    obstacles_fixed_(0),
    cylinders_fixed_(0),
    folder_path_(kitti_folder_path),
    subfolder_path_(subfolder_path),
    frame_(0),
    total_frames_(0),
    total_obstacles_(0)
  {
    frame_ = 0;
    total_frames_ = 0;

    if(folder_path_.front() != '/') folder_path_ = '/' + folder_path_;
    if(folder_path_.back() != '/') folder_path_ += '/';

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

  kittiCustomObjectsReader::~kittiCustomObjectsReader(){
    if(frame_connection_.connected())
      frame_connection_.disconnect();
    if(dataset_connection_.connected())
      dataset_connection_.disconnect();
  }

  bool kittiCustomObjectsReader::SetDataset(const unsigned int dataset_number){
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

      return true;
    }else
      std::cout << "\n\033[1;41m Error: \033[0;1;38;5;174m The folder: " << folder_path_
                << subfolder_path_ << " was not found. \033[0m\n" << std::endl;

    return false;
  }

  bool kittiCustomObjectsReader::GoToFrame(const unsigned int frame_number){
    obstacles_.clear();
    cylinders_.clear();
    obstacles_fixed_.clear();
    cylinders_fixed_.clear();
    total_obstacles_ = 0;
    frame_ = frame_number;

    std::string file_name("0000000000.txt");
    const std::string number(std::to_string(frame_number));
    file_name.replace(file_name.begin() + 10 - number.size(), file_name.begin() + 10, number);

    const std::string full_path(folder_path_ + subfolder_path_ + file_name);
    std::string line;

    // Open the text file
    std::ifstream file(full_path.c_str());
    if(file.is_open()){
      // Counting lines
      while(std::getline(file, line))
        if(line.size() <= 0)
          continue;
        else if(line[0] == '#')
          continue;
        else
          total_obstacles_++;

      // Returning to the file's beginning
      file.clear();
      file.seekg(0, file.beg);

      // Reserving space in std::vectors to avoid reallocations
      obstacles_.reserve(total_obstacles_);
      cylinders_.reserve(total_obstacles_);
      obstacles_fixed_.reserve(total_obstacles_);
      cylinders_fixed_.reserve(total_obstacles_);

      // Obtaining the data
      while(std::getline(file, line)){
        if(line.size() <= 0)
          continue;
        else if(line.front() == '#')
          continue;

        Visualizer::Object object;
        const char *line_content = line.c_str();
        algebraica::quaternionF quaternion;
        algebraica::vec3f euler;
        unsigned int object_type, which_frame, has_arrow;
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
        std::sscanf(line_content, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%u,%u,%u",
                    &object.width, &object.length, &object.height,
                    &object.position.x, &object.position.y, &object.position.z,
                    &euler[0], &euler[1], &euler[2],
                    &object.color.red, &object.color.green, &object.color.blue,
                    &object.line_width, &object_type, &which_frame, &has_arrow);

        // Calculating the orientation from Euler angles to quaternion
        quaternion.from_euler(euler[2], euler[1], euler[0]);

        object.orientation.x = object.arrow.orientation.x = quaternion.x();
        object.orientation.y = object.arrow.orientation.y = quaternion.y();
        object.orientation.z = object.arrow.orientation.z = quaternion.z();
        object.orientation.w = object.arrow.orientation.w = quaternion.w();
        object.arrow.display = has_arrow == 1;

        if(which_frame == 1)
          if(object_type == 1)
            cylinders_fixed_.push_back(object);
          else
            obstacles_fixed_.push_back(object);
        else
          if(object_type == 1)
            cylinders_.push_back(object);
          else
            obstacles_.push_back(object);
      }
      // Triggering the ready signal
      signal_();
      std::cout << "\n\033[1;30;46m Frame: \033[0;1;38;5;195m " << frame_
                << "\033[0m" << std::endl;

      return total_obstacles_ > 0;
    }else{
      std::cout << "\n\033[1;41m Error: \033[0;1;38;5;174m The file: " << full_path
                << " was not found. \033[0m\n" << std::endl;
      return false;
    }
  }

  const std::vector<Visualizer::Object> *const kittiCustomObjectsReader::GetObstacles(){
    return &obstacles_;
  }

  const std::vector<Visualizer::Object> *const kittiCustomObjectsReader::GetCylinders(){
    return &cylinders_;
  }

  const std::vector<Visualizer::Object> *const kittiCustomObjectsReader::GetObstaclesFixed(){
    return &obstacles_fixed_;
  }

  const std::vector<Visualizer::Object> *const kittiCustomObjectsReader::GetCylindersFixed(){
    return &cylinders_fixed_;
  }

  void kittiCustomObjectsReader::ConnectFrame(boost::signals2::signal<void (unsigned int)> *signal){
    // Disconnects if previously connected
    if(frame_connection_.connected())
      frame_connection_.disconnect();
    // Makes the new connection
    frame_connection_ =
        signal->connect(boost::bind(&kittiCustomObjectsReader::GoToFrame, this, _1));
  }

  void kittiCustomObjectsReader::ConnectDataset(boost::signals2::signal<void (unsigned int)> *signal){
    if(dataset_connection_.connected())
      dataset_connection_.disconnect();
    dataset_connection_ =
        signal->connect(boost::bind(&kittiCustomObjectsReader::SetDataset, this, _1));
  }

  boost::signals2::signal<void ()> *kittiCustomObjectsReader::Signal(){
    return &signal_;
  }

  const unsigned int kittiCustomObjectsReader::ActualFrame(){
    return frame_;
  }

  const unsigned int kittiCustomObjectsReader::TotalFrames(){
    return total_frames_;
  }
}
