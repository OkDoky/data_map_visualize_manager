#include <sstream>
#include <chrono>
#include <iostream>
#include "footprint_processor.h"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(dmvm::FootprintProcessor, ProcessorInterface)

namespace dmvm {
FootprintProcessor::FootprintProcessor(){}
void FootprintProcessor::initialize(const std::string& ns){
    nh_ = ros::NodeHandle(ns);
    ps_nh_ = ros::NodeHandle(ros::this_node::getName());
    ROS_INFO("[FootprintProcessor] ns : %s", ns.c_str());

    ps_nh_.param<std::string>("map_frame", base_frame_, "base_footprint");
    ROS_INFO("[FootprintProcessor] base frame : %s, processor node name : %s", base_frame_.c_str(), ros::this_node::getName().c_str());
    
    data_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("data_map", 1);

    nh_.param<float>("map_height", map_height_, 10.0);
    nh_.param<float>("map_width", map_width_, 10.0);
    nh_.param<float>("map_resolution", map_resolution_, 0.05);
    nh_.param<int>("update_rate", update_rate_, 10);
    nh_.param<bool>("use_padding", use_padding_, false);
    if (use_padding_){
        nh_.param<float>("padding_size", padding_size_, 0.1);
    }
    int data_value_int;
    nh_.param<int>("data_value", data_value_int, 64);
    data_value_ = static_cast<unsigned char>(data_value_int);
    if (update_rate_ != 0){
        sleep_rate_ = static_cast<float>(1.0/update_rate_);
    } else {
        sleep_rate_ = 0.1;
    }
    ROS_WARN("[FootprintProcessor] datamap will update every %.2f seconds", sleep_rate_);
    size_x = static_cast<unsigned int>(map_width_ / map_resolution_);
    size_y = static_cast<unsigned int>(map_height_ / map_resolution_);
    data_map_size_ = static_cast<size_t>(size_x * size_y);
    default_value_ = 0;
    data_map_.resize(data_map_size_, default_value_);

    fixed_origin_.position.x = -4.95;
    fixed_origin_.position.y = -4.95;
    fixed_origin_.orientation.w = 1.0;
    footprint_ = makeFootprintFromParams();
    processData(footprint_);
}

void FootprintProcessor::start(){
    return;
}

void FootprintProcessor::getCharMap(std::vector<unsigned char>& data_map) {
    // Lock the mutex before updating data_map_
    std::lock_guard<std::mutex> lock(data_map_mutex_);
    data_map = data_map_;
}

void FootprintProcessor::getOccupiedIndices(std::vector<size_t>& occupied_indices) {
    std::lock_guard<std::mutex> lock(data_map_mutex_);
    occupied_indices = occupied_indices_;
}

void FootprintProcessor::getDataValue(unsigned char& data_value) const{
  data_value = data_value_;
}

void FootprintProcessor::startStandAlone(){
    int sleep_rate_millis_ = static_cast<int>(sleep_rate_ * 1000.0);
    nav_msgs::OccupancyGrid dm; 
    updateDataMap(dm);
    while (ros::ok()){
        publishDataMap(dm);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_rate_millis_)); //
    }
}

void FootprintProcessor::processData(const std::vector<geometry_msgs::Point>& polygon){
    // make footprint polygon to be linked(e.g. square -> 5points)
    std::vector<geometry_msgs::Point> linked_footprint = polygon;
    linked_footprint.push_back(linked_footprint.front());

    // padding footprint
    if (use_padding_){
        padFootprint(linked_footprint);
    }

    // make interpolated as resolution 0.05 * 0.9 (map resolution)
    std::vector<geometry_msgs::Point> interpolated_footprint;
    interpolatedFootprint(linked_footprint, interpolated_footprint);
    
    // Lock the mutex before updating data_map_
    std::lock_guard<std::mutex> lock(data_map_mutex_);

    std::fill(data_map_.begin(), data_map_.end(), default_value_);
    occupied_indices_.clear();
    index_flags_.clear();

    // fill in interpolated footprint to datamap
    for (size_t i = 0; i < interpolated_footprint.size(); i++){
        unsigned int mx, my;
        if (!worldToMap(interpolated_footprint[i].x, interpolated_footprint[i].y, mx, my)) {
            continue;
        }
        unsigned int index = getIndex(mx, my);
        data_map_[index] = data_value_;
        if (index_flags_.find(index) == index_flags_.end()){
          occupied_indices_.push_back(index);
          index_flags_.insert(index);
        }
    }
    // // start thread for publish..
    // publish_thread_ = std::thread(&FootprintProcessor::publishThread, this);
}

void FootprintProcessor::interpolatedFootprint(const std::vector<geometry_msgs::Point>& footprint_edge,
                                               std::vector<geometry_msgs::Point>& interpolated_footprint){
    // footprint edge point(5points)
    for (size_t i = 1; i < footprint_edge.size(); i++){
        double dx, dy, dist;
        dx = footprint_edge[i].x - footprint_edge[i-1].x;
        dy = footprint_edge[i].y - footprint_edge[i-1].y;
        dist = hypot(dx, dy);
        int waypoints = static_cast<int>(dist/(map_resolution_ * 0.9));
        double increase_x = dx/waypoints;
        double increase_y = dy/waypoints;
        
        // interpolate 4points
        for (unsigned int j = 0; j < waypoints; j++){
            double px, py;
            geometry_msgs::Point p;
            p.x = footprint_edge[i-1].x + j * increase_x;
            p.y = footprint_edge[i-1].y + j * increase_y;
            interpolated_footprint.push_back(p);
        }

    }
}

void FootprintProcessor::padFootprint(std::vector<geometry_msgs::Point>& footprint){
  // pad footprint in place
  for (unsigned int i = 0; i < footprint.size(); i++)
  {
    geometry_msgs::Point& pt = footprint[ i ];
    pt.x += sign0(pt.x) * padding_size_;
    pt.y += sign0(pt.y) * padding_size_;
  }
}

void FootprintProcessor::updateDataMap(nav_msgs::OccupancyGrid& dm){
    //initialize data map
    dm.header.frame_id = base_frame_;
    dm.header.stamp = ros::Time::now();
    dm.info.resolution = map_resolution_;
    dm.info.width = size_x;
    dm.info.height = size_y;
    dm.info.origin = fixed_origin_;
    dm.data.resize(size_x * size_y);
    std::transform(data_map_.begin(), data_map_.end(), dm.data.begin(), [](unsigned char c){
        return static_cast<signed char>(c);
    });
}

void FootprintProcessor::publishDataMap(nav_msgs::OccupancyGrid& dm){
    dm.header.stamp = ros::Time::now();
    data_map_pub_.publish(dm);
}

bool FootprintProcessor::worldToMap(double wx, double wy,
                                      unsigned int& mx, unsigned int& my){
    if (wx < fixed_origin_.position.x || wy < fixed_origin_.position.y)
        return false;
    mx = static_cast<unsigned int>((wx - fixed_origin_.position.x) / map_resolution_);
    my = static_cast<unsigned int>((wy - fixed_origin_.position.y) / map_resolution_);
    return mx < size_x && my < size_y;
}

void FootprintProcessor::mapToWorld(unsigned int mx, unsigned int my,
                                      double& wx, double& wy){
    wx = fixed_origin_.position.x + (mx + 0.5) * map_resolution_;
    wy = fixed_origin_.position.y + (my + 0.5) * map_resolution_;
}


std::string FootprintProcessor::formatPolygon(const std::vector<geometry_msgs::Point>& polygon){
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < polygon.size(); ++i) {
        const auto& point = polygon[i];
        oss << "(" << point.x << ", " << point.y << ", " << point.z << ")";
        if (i < polygon.size() - 1) {
            oss << ", ";
        }
    }
    oss << "]";
    return oss.str();
}

std::vector<geometry_msgs::Point> FootprintProcessor::makeFootprintFromParams() {
  std::string full_param_name;
  std::string full_radius_param_name;
  std::vector<geometry_msgs::Point> points;

  if (nh_.searchParam("footprint", full_param_name)) {
    ROS_WARN("[makeFootprintFromPrams] succeded to search params..");
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    nh_.getParam(full_param_name, footprint_xmlrpc);
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString &&
        footprint_xmlrpc != "" && footprint_xmlrpc != "[]") {
      if (makeFootprintFromString(std::string(footprint_xmlrpc), points)) {
        writeFootprintToParam(points);
        return points;
      }
    } else if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      points = makeFootprintFromXMLRPC(footprint_xmlrpc, full_param_name);
      writeFootprintToParam(points);
      return points;
    }
  } else{
    ROS_WARN("[makeFootprintFromPrams] fail to search params..");
  }
  // Else neither param was found anywhere this knows about, so
  // defaults will come from dynamic_reconfigure stuff, set in
  // cfg/Costmap2D.cfg and read in this file in reconfigureCB().
  return points;
}

bool FootprintProcessor::makeFootprintFromString(const std::string& footprint_string, std::vector<geometry_msgs::Point>& footprint){
  std::string error;
  std::vector<std::vector<float> > vvf = parseVVF(footprint_string, error);

  if (error != "")
  {
    ROS_ERROR("Error parsing footprint parameter: '%s'", error.c_str());
    ROS_ERROR("  Footprint string was '%s'.", footprint_string.c_str());
    return false;
  }

  // convert vvf into points.
  if (vvf.size() < 3)
  {
    ROS_ERROR("You must specify at least three points for the robot footprint, reverting to previous footprint.");
    return false;
  }
  footprint.reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++)
  {
    if (vvf[ i ].size() == 2)
    {
      geometry_msgs::Point point;
      point.x = vvf[ i ][ 0 ];
      point.y = vvf[ i ][ 1 ];
      point.z = 0;
      footprint.push_back(point);
    }
    else
    {
      ROS_ERROR("Points in the footprint specification must be pairs of numbers.  Found a point with %d numbers.",
                 int(vvf[ i ].size()));
      return false;
    }
  }

  return true;
}

std::vector<geometry_msgs::Point> FootprintProcessor::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc,
                                const std::string& full_param_name){
  // Make sure we have an array of at least 3 elements.
  if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      footprint_xmlrpc.size() < 3) {
    ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
               full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
    throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                             "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }

  std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point pt;

  for (int i = 0; i < footprint_xmlrpc.size(); ++i) {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        point.size() != 2) {
      ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                 full_param_name.c_str());
      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y = getNumberFromXMLRPC(point[ 1 ], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

double FootprintProcessor::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name){
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble){
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
    throw std::runtime_error("Values in the footprint specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

void FootprintProcessor::writeFootprintToParam(const std::vector<geometry_msgs::Point>& footprint){
  std::ostringstream oss;
  bool first = true;
  for (unsigned int i = 0; i < footprint.size(); i++){
    geometry_msgs::Point p = footprint[ i ];
    if (first) {
      oss << "[[" << p.x << "," << p.y << "]";
      first = false;
    } else {
      oss << ",[" << p.x << "," << p.y << "]";
    }
  }
  oss << "]";
  nh_.setParam("footprint", oss.str().c_str());
}

} // namespace dmvm