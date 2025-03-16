

#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include<map>
#include <rcpputils/asserts.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
//#include "apriltag_ros/single_image_detector.h"

using namespace std::chrono_literals;

class StandaloneTagDescription
{
 public:
  StandaloneTagDescription() {};
  StandaloneTagDescription(int id, double size,
                           std::string &frame_name) :
      id_(id),
      size_(size),
      frame_name_(frame_name) {}

  double size() { return size_; }
  int id() { return id_; }
  std::string& frame_name() { return frame_name_; }

 private:
  // Tag description
  int id_;
  double size_;
  std::string frame_name_;
};

// Stores the properties of a tag member of a bundle
struct TagBundleMember
{
  int id; // Payload ID
  double size; // [m] Side length
  cv::Matx44d T_oi; // Rigid transform from tag i frame to bundle origin frame
};


class TagBundleDescription
{
 public:
  std::map<int, int > id2idx_; // (id2idx_[<tag ID>]=<index in tags_>) mapping

  TagBundleDescription(const std::string& name) :
      name_(name) {}

  void addMemberTag(int id, double size, cv::Matx44d T_oi) {
    TagBundleMember member;
    member.id = id;
    member.size = size;
    member.T_oi = T_oi;
    tags_.push_back(member);
    id2idx_[id] = tags_.size()-1;
  }

  const std::string& name() const { return name_; }
  // Get IDs of bundle member tags
  std::vector<int> bundleIds () {
    std::vector<int> ids;
    for (unsigned int i = 0; i < tags_.size(); i++) {
      ids.push_back(tags_[i].id);
    }
    return ids;
  }
  // Get sizes of bundle member tags
  std::vector<double> bundleSizes () {
    std::vector<double> sizes;
    for (unsigned int i = 0; i < tags_.size(); i++) {
      sizes.push_back(tags_[i].size);
    }
    return sizes;
  }
  int memberID (int tagID) { return tags_[id2idx_[tagID]].id; }
  double memberSize (int tagID) { return tags_[id2idx_[tagID]].size; }
  cv::Matx44d memberT_oi (int tagID) { return tags_[id2idx_[tagID]].T_oi; }

 private:
  // Bundle description
  std::string name_;
  std::vector<TagBundleMember > tags_;
};


class SingleImageServer : public rclcpp::Node
{
public:
  SingleImageServer(const std::string & name = "apriltag_ros_single_image_server",
    const std::string & namespace_ = "",
    const rclcpp::NodeOptions & options = (
      rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true)
  ))
  : Node(name, namespace_, options)
  {
    //standalone_tag_descriptions = this->declare_parameter("standalone_tags", std::vector<std::string>());
    //this->declare_parameter("my_parameter", "world");

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&SingleImageServer::timer_callback, this));
  }

  void timer_callback()
  {
    std::map<std::string, rclcpp::Parameter> standalonetags_map;
    this->get_parameters("standalone_tags", standalonetags_map);
    standalone_tag_descriptions = parseStandaloneTags(standalonetags_map);
    std::map<std::string, rclcpp::Parameter> tagbundles_map;
    this->get_parameters("tag_bundles", tagbundles_map);
    parseTagBundles(tagbundles_map);
    //for ( auto tag : tags)
    //	RCLCPP_INFO(this->get_logger(), "My standalone_tag is %s", tag.c_str());

    //std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    //this->set_parameters(all_new_parameters);
  }

private:
  std::map<int, StandaloneTagDescription> standalone_tag_descriptions;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<int, StandaloneTagDescription> parseStandaloneTags (
    std::map<std::string, rclcpp::Parameter> & standalone_tags);
  std::vector<TagBundleDescription > parseTagBundles (
    std::map<std::string, rclcpp::Parameter> & tag_bundles);
  double GetDoubleWithDefault (rclcpp::Parameter& param,
                                                std::string field,
                                                double defaultValue) const;
double GetDouble (rclcpp::Parameter& param,
                                     std::string field) const;
bool findStandaloneTagDescription (
    int id, StandaloneTagDescription*& descriptionContainer, bool printWarning);
};

bool SingleImageServer::findStandaloneTagDescription (
    int id, StandaloneTagDescription*& descriptionContainer, bool printWarning)
{
  std::map<int, StandaloneTagDescription>::iterator description_itr =
      standalone_tag_descriptions.find(id);
  if (description_itr == standalone_tag_descriptions.end())
  {
    if (printWarning)
    {
      rclcpp::Clock clock = rclcpp::Clock();
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("rclcpp"), clock, 10.0, "Requested description of standalone tag ID [%d],"
                        " but no description was found...",id);
    }
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "size = %f id = %d frame name = %s", description_itr->second.size(), description_itr->second.id(), description_itr->second.frame_name().c_str());
  descriptionContainer = &(description_itr->second);
  return true;
}

// parse tag bundle descriptions
std::vector<TagBundleDescription > SingleImageServer::parseTagBundles (
    std::map<std::string, rclcpp::Parameter> & tag_bundles)
{
  std::vector<TagBundleDescription > descriptions;
  //rcpputils::assert_true(tag_bundles.getType() == XmlRpc::XmlRpcValue::TypeArray);
  auto tag_bundles_it = tag_bundles.begin();
  std::map<std::string, std::vector<rclcpp::Parameter>> bundle_map;
  
  int b_num = 0;
  while ( tag_bundles_it != tag_bundles.end())
  {
    std::string bundle_num = tag_bundles_it->first.substr(0, tag_bundles_it->first.find_first_of("."));
    std::string int_bundle_num = bundle_num;
    
    auto actual_name_it = tag_bundles.find(bundle_num + ".name");
    std::string name = bundle_num;
    if (actual_name_it != tag_bundles.end())
    {
       name = actual_name_it->second.as_string();
    }
    
    TagBundleDescription bundle_i(name);
    while(bundle_num == int_bundle_num)
    {
      if (tag_bundles_it->first == int_bundle_num + ".name")
      {
        ++tag_bundles_it;
        if (tag_bundles_it == tag_bundles.end()) break;
        int_bundle_num = tag_bundles_it->first.substr(0, tag_bundles_it->first.find_first_of("."));
        continue;
      }
      int tag_start_pos = tag_bundles_it->first.find("layout.") + 7;
      int tag_end_pos = tag_bundles_it->first.find_first_of(".", tag_start_pos);
      std::string tag_num = tag_bundles_it->first.substr(tag_start_pos, tag_end_pos - tag_start_pos);
      std::string int_tag_num = tag_num;
      int id;
      double size = 0.;
      double x = 0.;
      double y = 0.;
      double z = 0.;
      double qw = 1.;
      double qx = 0.;
      double qy = 0.;
      double qz = 0.;
      
      int t_num = 0;
      while(bundle_num == int_bundle_num && tag_num == int_tag_num)
      {
        rclcpp::Parameter param = tag_bundles_it->second;
        std::string param_name = tag_bundles_it->second.get_name();
        if(param.get_name().find("id") != std::string::npos)
        {
          rcpputils::assert_true(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER );
          id = param.as_int();
        }
        size = GetDoubleWithDefault(param, "size", size);
        x = GetDoubleWithDefault(param, "x", x);
        y = GetDoubleWithDefault(param, "y", y);
        z = GetDoubleWithDefault(param, "z", z);
        qw = GetDoubleWithDefault(param, "qw", qw);
        qx = GetDoubleWithDefault(param, "qx", qx);
        qy = GetDoubleWithDefault(param, "qy", qy);
        qz = GetDoubleWithDefault(param, "qz", qz);
      
        ++tag_bundles_it;
        if (tag_bundles_it == tag_bundles.end()) break;
        int_bundle_num = tag_bundles_it->first.substr(0, tag_bundles_it->first.find_first_of("."));
        tag_start_pos = tag_bundles_it->first.find("layout.") + 7;
        tag_end_pos = tag_bundles_it->first.find_first_of(".", tag_start_pos);
        int_tag_num = tag_bundles_it->first.substr(tag_start_pos, tag_end_pos - tag_start_pos);
      }
      
      // Make sure that if this tag was specified also as standalone,
      // then the sizes match
      StandaloneTagDescription* standaloneDescription;
      if (findStandaloneTagDescription(id, standaloneDescription, false))
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Bsize = %f std sizwee() = %f ", size, standaloneDescription->size());
        rcpputils::assert_true(size == standaloneDescription->size());
      }
      
      Eigen::Quaterniond q_tag(qw, qx, qy, qz);
      q_tag.normalize();
      Eigen::Matrix3d R_oi = q_tag.toRotationMatrix();

      // Build the rigid transform from tag_j to the bundle origin
      cv::Matx44d T_mj(R_oi(0,0), R_oi(0,1), R_oi(0,2), x,
                       R_oi(1,0), R_oi(1,1), R_oi(1,2), y,
                       R_oi(2,0), R_oi(2,1), R_oi(2,2), z,
                       0,         0,         0,         1);
                       
      bundle_i.addMemberTag(id, size, T_mj);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp")," " << t_num << ") id: " << id << ", size: " << size << ", "
                          << "p = [" << x << "," << y << "," << z << "], "
                          << "q = [" << qw << "," << qx << "," << qy << ","
                          << qz << "]");
      //++tag_bundles_it;
      ++t_num;
      if (tag_bundles_it == tag_bundles.end()) break;
      int_bundle_num = tag_bundles_it->first.substr(0, tag_bundles_it->first.find_first_of("."));
    }
    
    ++b_num;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Bundle Number %d logging done ", b_num);
    descriptions.push_back(bundle_i);
  }
/*
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Number of Bundles %d ", (int)descriptions.size());
  for (auto des : descriptions)
  {
    std::vector<int> ids = des.bundleIds();
    for (auto id : ids)
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Bundle Name %s, tag id = %d ", des.name().c_str(), id);
  }
*/
  return descriptions;
}

double SingleImageServer::GetDouble (rclcpp::Parameter& param,
                                     std::string field) const
{
  rcpputils::assert_true(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ||
             (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER));
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    int tmp = param.as_int();
    return (double)tmp;
  }
  else
  {
    return param.as_double();
  }
}

double SingleImageServer::GetDoubleWithDefault (rclcpp::Parameter& param,
                                                std::string field,
                                                double defaultValue) const
{
  if (param.get_name().find("." + field) != std::string::npos)
  {
    rcpputils::assert_true(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ||
        (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER));
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      int tmp = param.as_int();
      return (double)tmp;
    }
    else
    {
      return param.as_double();
    }
  }
  else
  {
    return defaultValue;
  }
}

// Parse standalone tag descriptions
std::map<int, StandaloneTagDescription> SingleImageServer::parseStandaloneTags (
    std::map<std::string, rclcpp::Parameter> & standalone_tags)
{
  // Create map that will be filled by the function and returned in the end
  std::map<int, StandaloneTagDescription> descriptions;
  // Ensure the type is correct
  //ROS_ASSERT(standalone_tags.getType() == XmlRpc::XmlRpcValue::TypeArray);
  // Loop through all tag descriptions
  auto map_it = standalone_tags.begin();
  
  while ( map_it != standalone_tags.end())
  {
    std::vector<rclcpp::Parameter> params;
    std::string tag_name = map_it->first.substr(0, map_it->first.find_first_of("."));
    auto it = map_it; 
    while (it != standalone_tags.end())
    {
      std::string tag_full_name = it->second.get_name();
      if (tag_full_name.find(tag_name) != std::string::npos)
      {
        params.push_back(it->second);
      }
      else
        break;
      ++it;
    }
    map_it = it;
    
    int id = 0;
    double size = 0.0;
    std::string frame_name = "";
    
    for (auto param : params)
    {
      std::string param_name = param.get_name();
      if(param_name.find("id") != std::string::npos)
      {
        rcpputils::assert_true(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER );
        id = param.as_int();
      }
      if(param_name.find("name") != std::string::npos)
      {
        rcpputils::assert_true(param.get_type() == rclcpp::ParameterType::PARAMETER_STRING );
        frame_name = param.as_string();
      }
      if(param_name.find("size") != std::string::npos)
      {
        rcpputils::assert_true(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE );
        size = param.as_double();
      }
    }
    
    StandaloneTagDescription description(id, size, frame_name);
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded tag config: " << id << ", size: " <<
                    size << ", frame_name: " << frame_name.c_str());
    // Add this tag's description to map of descriptions
    descriptions.insert(std::make_pair(id, description));
  }

  return descriptions;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SingleImageServer>());
  rclcpp::shutdown();
  return 0;
}

struct MyStruct {
  int int_value;
  double double_value;
  std::string string_value;
};

void declare_struct_parameters(rclcpp::Node::SharedPtr node, const std::string& prefix = "") {
  node->declare_parameter(prefix + "int_value", 0);
  node->declare_parameter(prefix + "double_value", 0.0);
  node->declare_parameter(prefix + "string_value", "");
}

void set_struct_parameters(rclcpp::Node::SharedPtr node, const MyStruct& my_struct, const std::string& prefix = "") {
  node->set_parameter(rclcpp::Parameter(prefix + "int_value", my_struct.int_value));
  node->set_parameter(rclcpp::Parameter(prefix + "double_value", my_struct.double_value));
  node->set_parameter(rclcpp::Parameter(prefix + "string_value", my_struct.string_value));
}

MyStruct get_struct_parameters(rclcpp::Node::SharedPtr node, const std::string& prefix = "") {
  MyStruct my_struct;
  my_struct.int_value = node->get_parameter(prefix + "int_value").as_int();
  my_struct.double_value = node->get_parameter(prefix + "double_value").as_double();
  my_struct.string_value = node->get_parameter(prefix + "string_value").as_string();
  return my_struct;
}
/*
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  //ros::init(argc, argv, "apriltag_ros_single_image_server");
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("apriltag_ros_single_image_server");
  // Parse standalone tag descriptions specified by user (stored on ROS
  // parameter server)

  
  std::vector<std::string> standalone_tag_descriptions = node->declare_parameter<std::vector<std::string>>("standalone_tags", std::vector<std::string>());
  if(standalone_tag_descriptions.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "No april tags specified");
  }
  else
  {
    try
    {
       // Load each controller
       for (auto tag : standalone_tag_descriptions)
       {
         RCLCPP_INFO(node->get_logger(), "Loading %s", tag.c_str());
         //load(controller_name);
       }
      //standalone_tag_descriptions_ =
          //parseStandaloneTags(standalone_tag_descriptions);
    }
    catch(const rclcpp::ParameterTypeException& e)
    {
      // in case any of the asserts in parseStandaloneTags() fail
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error loading standalone tag descriptions: ");
    }
  }
  //ros::NodeHandle nh;
  //ros::NodeHandle pnh("~");

  //apriltag_ros::SingleImageDetector continuous_tag_detector(nh, pnh);
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STarted and Ended!!!!");
  rclcpp::shutdown();
  return 0;
}
*/

/*
// Parse standalone tag descriptions
std::map<int, StandaloneTagDescription> TagDetector::parseStandaloneTags (
    XmlRpc::XmlRpcValue& standalone_tags)
{
  // Create map that will be filled by the function and returned in the end
  std::map<int, StandaloneTagDescription> descriptions;
  // Ensure the type is correct
  ROS_ASSERT(standalone_tags.getType() == XmlRpc::XmlRpcValue::TypeArray);
  // Loop through all tag descriptions
  for (int32_t i = 0; i < standalone_tags.size(); i++)
  {

    // i-th tag description
    XmlRpc::XmlRpcValue& tag_description = standalone_tags[i];

    // Assert the tag description is a struct
    ROS_ASSERT(tag_description.getType() ==
               XmlRpc::XmlRpcValue::TypeStruct);
    // Assert type of field "id" is an int
    ROS_ASSERT(tag_description["id"].getType() ==
               XmlRpc::XmlRpcValue::TypeInt);
    // Assert type of field "size" is a double
    ROS_ASSERT(tag_description["size"].getType() ==
               XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"]; // tag id
    // Tag size (square, side length in meters)
    double size = (double)tag_description["size"];

    // Custom frame name, if such a field exists for this tag
    std::string frame_name;
    if(tag_description.hasMember("name"))
    {
      // Assert type of field "name" is a string
      ROS_ASSERT(tag_description["name"].getType() ==
                 XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["name"];
    }
    else
    {
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }

    StandaloneTagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: " << id << ", size: " <<
                    size << ", frame_name: " << frame_name.c_str());
    // Add this tag's description to map of descriptions
    descriptions.insert(std::make_pair(id, description));
  }

  return descriptions;
}
*/
