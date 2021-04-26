#include "kitti/kitti_vehicle_reader.h"
#include "algebraica/algebraica.h"

#include <locale>
#include <codecvt>

namespace Kitti {
  KittiVehicleReader::KittiVehicleReader(const std::string folder_path) :
    vehicle_{ { 0.0f, 0.0f, 0.0f }, // position
              { 0.0f, 0.0f, 0.0f }, // position_xyz
              { 0.0f, 0.0f, 0.0f }, // velocity
              { 0.0f, 0.0f, 0.0f }, // acceleration
              { 0.0f, 0.0f, 0.0f, 0.0f }, // orientation
              { 0.0f, 0.0f, 0.0f }, // euler angles
              0.0f, 0.0f, 0.0f, // steering, speed, rpm
              0.0f, 0.0f, 0.0f, // fuel, gas, clutch
              0.0f, "P" }, // brake, gear
    timestamp_(),
    folder_path_(folder_path),
    frame_(0),
    total_frames_(0)
  {
    if(folder_path_.front() != '/') folder_path_ = "/" + folder_path_;
    if(folder_path_.back() != '/') folder_path_ += "/";

    const std::string first_path(folder_path_);

    if(!boost::filesystem::exists(boost::filesystem::path(folder_path_))){
      folder_path_ = boost::filesystem::current_path().string() + folder_path_;

      if(!boost::filesystem::exists(boost::filesystem::path(folder_path_)))
        std::cout << "The file: " << first_path << " was not found.\n" <<
                     "  Neither: " << folder_path_ << "\n";
    }

    folder_path_ += "raw/0000_sync/oxts/";

    /*
    std::cout << boost::filesystem::current_path().string() << std::endl;

    std::ifstream file(std::string(boost::filesystem::current_path().string() + "/SAM").c_str(),
                       std::ios::binary);
//    std::ofstream ofile(boost::filesystem::current_path().string() + "/SAM.txt");

    if(file.is_open()){
      std::wstring_convert<std::codecvt_utf8<char16_t>, char16_t> convert;
      std::u16string text;
      char16_t character[1];
//      char temporal[255];
//      file.read(&temporal[0], 3);
      while(file.read((char*)character, 2)){
//        character[0] = bytes[0] | (bytes[1] << 8);
        if(character[0] == u':')
          std::cout << "\033[1;31m : \033[0m";
//        else if(character[0] == u'\a')
//          std::cout << "\033[1;34m alarm \033[0m";
//        else if(character[0] == u'\b')
//          std::cout << "\033[1;36m backspace \033[0m";
//        else if(character[0] == u'\r')
//          std::cout << "\033[1;35m return \033[0m";
//        else if(character[0] == u'\t')
//          std::cout << "\033[1;33m tab \033[0m";
//        else if(character[0] == u'ü')
//          std::cout << "\033[1;33m urlaub \033[0m";
//        else if(character[0] == u'\v')
//          std::cout << "\033[1;30m vertical tab \033[0m";
//        else if(character[0] == u'\n')
//          std::cout << "\033[1;32mnew line\033[0m\n";
//        else if(character[0] == u'@')
//          std::cout << "\033[1;41m @ \033[0m";
//        else if(character[0] == u'\0')
//          std::cout << "\033[1;46m end \033[0m";
//        else
//          std::cout << convert.to_bytes(character);
//        text.push_back(character);
      }
//      ofile << text;
      std::cout << " : " << std::endl;
      file.close();
//      ofile.close();
    }else{
      std::cout << "file not open" << std::endl;
    }

    std::cout << "\n–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––\n"
              << std::flush;
    */
  }

  KittiVehicleReader::~KittiVehicleReader(){
    if(frame_connection_.connected())
      frame_connection_.disconnect();
    if(dataset_connection_.connected())
      dataset_connection_.disconnect();
  }

  bool KittiVehicleReader::set_dataset(const unsigned int dataset_number){
    bool result = false;

    std::string dataset(std::to_string(dataset_number));
    folder_path_.replace(folder_path_.end() - 11 - static_cast<int>(dataset.size()),
                         folder_path_.end() - 11, dataset);

    frame_ = 0;
    total_frames_ = 0;

    //opening the folder
    boost::filesystem::path directory(folder_path_ + "data/");

    if(boost::filesystem::exists(directory)){
      //viewing all the files inside the folder
      for(const boost::filesystem::directory_entry &entry :
          boost::filesystem::directory_iterator(directory))
        if(boost::filesystem::is_regular_file(entry.path()))
          total_frames_++;

      result = true;
    }

    return result;
  }

  bool KittiVehicleReader::go_to_frame(const unsigned int frame_number){
    bool result = false;

    //check if the selected frame number is bigger than existing frames
    if(frame_number < total_frames_){
      //this creates the file name string: if frame is 1 then the resulting name is
      //0000000001.bin
      frame_ = frame_number;
      std::string file_name("0000000000.txt");
      std::string number(std::to_string(frame_number));
      file_name.replace(file_name.begin() + 10 - static_cast<int>(number.size()),
                        file_name.begin() + 10, number);

      //open the binary file of the velodyne_points folder
      std::ifstream file(folder_path_ + "data/" + file_name);
      std::string line;

      if(file.is_open()){
        float acceleration_x{0.0f}, acceleration_y{0.0f};
        while(std::getline(file, line)){
          const char *line_c = line.c_str();
          std::sscanf(line_c, "%f %f %f %f %f %f %*f %*f %f %f %f %f %f %f",
                      &vehicle_.position.coordinates.latitude,
                      &vehicle_.position.coordinates.longitude,
                      &vehicle_.position.coordinates.altitude,
                      &vehicle_.euler.angles.roll, &vehicle_.euler.angles.pitch,
                      &vehicle_.euler.angles.yaw, &vehicle_.velocity.point.x,
                      &vehicle_.velocity.point.y, &vehicle_.velocity.point.z,
                      &acceleration_x, &acceleration_y, &vehicle_.acceleration.point.z);
        }

        const algebraica::quaternionF orientation{
          algebraica::quaternionF::euler_to_quaternion(static_cast<float>(vehicle_.euler.angles.pitch),
                                                       static_cast<float>(vehicle_.euler.angles.yaw),
                                                       static_cast<float>(vehicle_.euler.angles.roll))
        };
        vehicle_.orientation.axes.x = orientation.x;
        vehicle_.orientation.axes.y = orientation.y;
        vehicle_.orientation.axes.z = orientation.z;
        vehicle_.orientation.axes.w = orientation.w;

        vehicle_.speed = std::sqrt(vehicle_.velocity.point.x * vehicle_.velocity.point.x +
                                   vehicle_.velocity.point.y * vehicle_.velocity.point.y +
                                   vehicle_.velocity.point.z * vehicle_.velocity.point.z) * 3.6f;
        vehicle_.rpm = 800.0f + vehicle_.speed * 85;

        vehicle_.acceleration.point.x += (acceleration_x - vehicle_.acceleration.point.x) * 0.3f;
        vehicle_.acceleration.point.y += (acceleration_y * 0.5f
                                          - vehicle_.acceleration.point.y) * 0.1f;

        vehicle_.gas = (vehicle_.acceleration.point.x > 0.0f)?
                         std::abs(vehicle_.acceleration.point.x / 5.0f) : 0.0f;
        vehicle_.brake = (vehicle_.acceleration.point.x < 0.0f)?
                           std::abs(vehicle_.acceleration.point.x / 5.0f) : 0.0f;
        vehicle_.steering_angle = 0.9f * static_cast<float>(vehicle_.acceleration.point.y);
        vehicle_.clutch = 0.4f;

        if(vehicle_.speed < 0.0f)
          vehicle_.gear = "R";
        else if(vehicle_.speed < 20)
          vehicle_.gear = "1";
        else if(vehicle_.speed < 40)
          vehicle_.gear = "2";
        else if(vehicle_.speed < 60)
          vehicle_.gear = "3";
        else if(vehicle_.speed < 80)
          vehicle_.gear = "4";
        else if(vehicle_.speed < 100)
          vehicle_.gear = "5";

        //close the opened file
        file.close();
        result = true;

        signal_();
      }else
        //if an error occurs opening the file then this line will pop up
        std::cout << "the file: " << folder_path_ + file_name << " was not found." << std::endl;

      std::ifstream file_t(std::string(folder_path_ + "timestamps.txt").c_str());
      if(file_t.is_open()){
        for(unsigned int i = 0; i <= frame_; i++)
          std::getline(file_t, timestamp_, '\n');

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

  void KittiVehicleReader::connect_frame(boost::signals2::signal<void (unsigned int)> &signal){
    if(frame_connection_.connected())
      frame_connection_.disconnect();
    frame_connection_ = signal.connect(boost::bind(&KittiVehicleReader::go_to_frame, this, _1));
  }

  void KittiVehicleReader::connect_dataset(boost::signals2::signal<void (unsigned int)> &signal){
    if(dataset_connection_.connected())
      dataset_connection_.disconnect();
    dataset_connection_ = signal.connect(boost::bind(&KittiVehicleReader::set_dataset, this, _1));
  }

  boost::signals2::signal<void ()> *KittiVehicleReader::signal(){
    return &signal_;
  }

  unsigned int KittiVehicleReader::actual_frame() const{
    return frame_;
  }

  unsigned int KittiVehicleReader::total_frames() const{
    return total_frames_;
  }

  const torero::Vehicle *KittiVehicleReader::vehicle() const{
    return &vehicle_;
  }

  const std::string &KittiVehicleReader::timestamp() const{
    return timestamp_;
  }

  const torero::OrientationPYR *KittiVehicleReader::euler_angles() const{
    return &vehicle_.euler;
  }

  const torero::OrientationXYZW *KittiVehicleReader::quaternion() const{
    return &vehicle_.orientation;
  }
}
