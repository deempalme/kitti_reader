#include "kitti/kitti_objects_reader.h"
// XML reader
#include "ticpp-master/ticpp.h"

namespace Kitti {
  KittiObjectsReader::KittiObjectsReader(const std::string folder_path) :
    all_obstacles_(0),
    obstacles_(0),
    folder_path_(folder_path),
    frame_(0),
    total_obstacles_(0)
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

    folder_path_ += "raw/0000_sync/tracklet_labels.xml";
  }

  KittiObjectsReader::~KittiObjectsReader(){
    if(frame_connection_.connected())
      frame_connection_.disconnect();
    if(dataset_connection_.connected())
      dataset_connection_.disconnect();
  }

  const unsigned int KittiObjectsReader::ActualFrame(){
    return frame_;
  }

  const unsigned int KittiObjectsReader::TotalObstacles(){
    return total_obstacles_;
  }

  const std::vector<Visualizer::Object> *KittiObjectsReader::GetObstacles() const{
    return &obstacles_;
  }

  bool KittiObjectsReader::SetDataset(const unsigned int dataset_number){

    const std::string dataset(std::to_string(dataset_number));
    folder_path_.replace(folder_path_.end() - 25 - dataset.size(), folder_path_.end() - 25, dataset);

    frame_ = 0;

    obstacles_.clear();
    all_obstacles_.clear();

    //opening the folder
    boost::filesystem::path directory(folder_path_);

    if(boost::filesystem::exists(directory)){
      ticpp::Document document;
      document.LoadFile(folder_path_);

      ticpp::Node *item = document.FirstChild("boost_serialization", false)->FirstChild(false)
                          ->FirstChild("item", false);

      std::cout << std::setprecision(3) << std::fixed;

      while(item){
        Kitti::Obstacles datum;

        datum.first_frame = std::stoi(item->FirstChildElement("first_frame", false)->GetText());
        datum.last_frame = datum.first_frame + std::stoi(item->FirstChild("poses", false)
                                                         ->FirstChildElement("count", false)
                                                         ->GetText());
        const std::string type(item->FirstChildElement(false)->GetText());

        if(type == "Car")
          datum.type = 0;
        else if(type == "Pedestrian")
          datum.type = 1;
        else if(type == "Cyclist")
          datum.type = 2;
        else if(type == "Van")
          datum.type = 3;
        else if(type == "Truck")
          datum.type = 4;
        else if(type == "Misc")
          datum.type = 5;

        datum.dimension.x = std::stof(item->FirstChildElement("l", false)->GetText());
        datum.dimension.y = std::stof(item->FirstChildElement("w", false)->GetText());
        datum.dimension.z = std::stof(item->FirstChildElement("h", false)->GetText());

        ticpp::Node *items = item->FirstChild("poses", false)->FirstChild("item", false);

        while(items){
          Algebraica::vec3f position;
          position[0] = std::stof(items->FirstChildElement("tx", false)->GetText());
          position[1] = std::stof(items->FirstChildElement("ty", false)->GetText());
          position[2] = std::stof(items->FirstChildElement("tz", false)->GetText());
          datum.position.push_back(position);

          Algebraica::quaternionF orientation =
              Algebraica::quaternionF::from_axis_and_angle(1.0f, 0.0f, 0.0f,-std::stof(items->FirstChildElement("ry", false)->GetText())) *
              Algebraica::quaternionF::from_axis_and_angle(0.0f, 1.0f, 0.0f,-std::stof(items->FirstChildElement("rx", false)->GetText())) *
              Algebraica::quaternionF::from_axis_and_angle(0.0f, 0.0f, 1.0f, std::stof(items->FirstChildElement("rz", false)->GetText()));
          datum.orientation.push_back(orientation);

          items = items->NextSibling("item", false);
        }
        all_obstacles_.push_back(datum);

        item = item->NextSibling("item", false);
      }
      return true;
    }else
      std::cout << "KitttiObjectsReader ---------------------\n  The file: "
                << folder_path_ << " was not found." << std::endl;

    return false;
  }

  bool KittiObjectsReader::GoToFrame(const unsigned int frame_number){

    //clear all the laser cloud information existing
    obstacles_.clear();
    obstacles_.reserve(30);
    //creates temporal variables out of the loop to avoid creating them in each tracklet
    Visualizer::Object datum;
    datum.name = "O";
    // Total number of obstacles in all frames
    const std::size_t total{all_obstacles_.size()};

    //go through all the obstacles and choose the ones that appear in this frame
    for(std::size_t i = 0; i < total; i++){
      //checks if this tracklet appears in this frame
      if(frame_number >= all_obstacles_[i].first_frame &&
         frame_number < all_obstacles_[i].last_frame){
        const std::size_t frame = static_cast<std::size_t>(frame_number)
                                  - all_obstacles_[i].first_frame;

        //temporal for color
        Algebraica::vec3f mC;

        switch(all_obstacles_[i].type){
        case 0:
          mC(5.0f, 164.0f, 217.0f);
          datum.line_width = 0.10f;
        break;
        case 1:
          mC(255.0f, 13.0f, 0.0f);
          datum.line_width = 0.07f;
        break;
        case 2:
          mC(255.0f, 122.0f, 0.0f);
          datum.line_width = 0.07f;
        break;
        case 3:
          mC(36.0f, 17.0f, 224.0f);
          datum.line_width = 0.13f;
        break;
        case 4:
          mC(225.0f, 0.0f, 189.0f);
          datum.line_width = 0.15f;
        break;
        case 5:
          mC(139.0f, 246.0f, 0.0f);
          datum.line_width = 0.10f;
        break;
        default:
          mC(170.0f, 203.0f, 207.0f);
          datum.line_width = 0.10f;
        break;
        }

        //the position changes every frame, so we need to go to the selected frame
        // and colect its position
        datum.position.x = all_obstacles_[i].position[frame][0];
        datum.position.y = all_obstacles_[i].position[frame][1];
        datum.position.z = all_obstacles_[i].position[frame][2];
        datum.position.z -= datum.position.z/2.0f;

        datum.length = datum.arrow.length = all_obstacles_[i].dimension.x;
        datum.width = all_obstacles_[i].dimension.y;
        datum.height = all_obstacles_[i].dimension.z;

        datum.orientation.x = datum.arrow.orientation.x = all_obstacles_[i].orientation[frame][0];
        datum.orientation.y = datum.arrow.orientation.y = all_obstacles_[i].orientation[frame][1];
        datum.orientation.z = datum.arrow.orientation.z = all_obstacles_[i].orientation[frame][2];
        datum.orientation.w = datum.arrow.orientation.w = all_obstacles_[i].orientation[frame][3];

        datum.color.red   = mC[0];
        datum.color.green = mC[1];
        datum.color.blue  = mC[2];

        //adding this obstacle to the vector
        obstacles_.push_back(datum);
      }
    }

    total_obstacles_ = obstacles_.size();
    signal_();

    return total_obstacles_ > 0;
  }

  void KittiObjectsReader::ConnectFrame(boost::signals2::signal<void (unsigned int)> *signal){
    if(frame_connection_.connected())
      frame_connection_.disconnect();
    frame_connection_ = signal->connect(boost::bind(&KittiObjectsReader::GoToFrame, this, _1));
  }

  void KittiObjectsReader::ConnectDataset(boost::signals2::signal<void (unsigned int)> *signal){
    if(dataset_connection_.connected())
      dataset_connection_.disconnect();
    dataset_connection_ = signal->connect(boost::bind(&KittiObjectsReader::SetDataset, this, _1));
  }

  boost::signals2::signal<void ()> *KittiObjectsReader::Signal(){
    return &signal_;
  }
}
