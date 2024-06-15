#include <chrono>
#include <iostream>
#include "velocity_processor.h"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(dmvm::VelocityProcessor, ProcessorInterface)

namespace dmvm {
VelocityProcessor::VelocityProcessor(){}

void VelocityProcessor::initialize(const std::string& ns){
    ROS_INFO("[VelocityProcessor] ns : %s", ns.c_str());
    private_nh_ = ros::NodeHandle(ns);
    nh_ = ros::NodeHandle("");
    private_nh_.param<std::string>("velocity_topic", velocity_topic_, "cmd_vel");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_footprint");
    private_nh_.param<bool>("stand_alone", stand_alone_, false);
    private_nh_.param<double>("arrow_scale", arrow_scale_, 1.0);
    subscriber_ = nh_.subscribe(velocity_topic_, 1, &VelocityProcessor::onVelocityReceived, this);
    if (stand_alone_){
        data_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("data_map", 1);
    }

    private_nh_.param<float>("map_height", map_height_, 10.0);
    private_nh_.param<float>("map_width", map_width_, 10.0);
    private_nh_.param<float>("map_resolution", map_resolution_, 0.05);
    int data_value_int;
    private_nh_.param<int>("data_value", data_value_int, 64);
    data_value_ = static_cast<unsigned char>(data_value_int);
    size_x = static_cast<unsigned int>(map_width_ / map_resolution_);
    size_y = static_cast<unsigned int>(map_height_ / map_resolution_);
    data_map_size_ = static_cast<size_t>(size_x * size_y);
    default_value_ = 0;
    data_map_.resize(data_map_size_, default_value_);

    fixed_origin_.position.x = -map_width_ / 2.0 + map_resolution_;
    fixed_origin_.position.y = -map_height_ / 2.0 + map_resolution_;
    fixed_origin_.orientation.w = 1.0;

    velocity_observable_ = velocity_subject_.get_observable();
    velocity_observable_.subscribe([this](const geometry_msgs::Twist& msg) {
        this->processData(msg);
    });
}

void VelocityProcessor::start(){
    ros::spin();
}

void VelocityProcessor::onVelocityReceived(const geometry_msgs::Twist::ConstPtr& msg){
    velocity_subject_.get_subscriber().on_next(*msg);
}

void VelocityProcessor::processData(const geometry_msgs::Twist& vel){
    // only for differencital drive robot
    double vec_linear = vel.linear.x;
    double vec_angular = vel.angular.z;

    double vec_x = vec_linear * cos(vec_angular) * arrow_scale_;
    double vec_y = vec_linear * sin(vec_angular) * arrow_scale_;

    double dist = hypot(vec_x, vec_y);
    int waypoints = static_cast<int>(dist/(map_resolution_ * 0.9));
    double increase_x = vec_x / waypoints;
    double increase_y = vec_y / waypoints;
    
    std::vector<Point> interpolated_points; 
    // interpolate 4points
    for (unsigned int j = 0; j < waypoints; j++){
        float px = j * increase_x;
        float py = j * increase_y;
        interpolated_points.push_back({px, py});
    }

    // Lock the mutex before updating data_map_
    std::lock_guard<std::mutex> lock(data_map_mutex_);

    // Resize and Reset data_map_
    std::fill(data_map_.begin(), data_map_.end(), default_value_);
    occupied_indices_.clear();
    index_flags_.clear();

    // fill data map
    for (size_t i = 0; i < interpolated_points.size(); i++) {
        float px = interpolated_points[i].px;
        float py = interpolated_points[i].py;

        unsigned int mx, my;
        if (!worldToMap(px, py, mx, my)) {
            continue;
        }
        unsigned int index = getIndex(mx, my);
        data_map_[index] = data_value_;
        if (index_flags_.find(index) == index_flags_.end()){
          occupied_indices_.push_back(index);
          index_flags_.insert(index);
        }

    }
    if (stand_alone_){
        publishDataMap(data_map_);
    }
}

void VelocityProcessor::getCharMap(std::vector<unsigned char>& data_map) {
    // Lock the mutex before accessing data_map_
    std::lock_guard<std::mutex> lock(data_map_mutex_);
    data_map = data_map_;
}

void VelocityProcessor::getOccupiedIndices(std::vector<size_t>& occupied_indices) {
    std::lock_guard<std::mutex> lock(data_map_mutex_);
    occupied_indices = occupied_indices_;
}

void VelocityProcessor::getDataValue(unsigned char& data_value) const{
  data_value = data_value_;
}

void VelocityProcessor::publishDataMap(const std::vector<unsigned char>& data){
    //initialize data map
    nav_msgs::OccupancyGrid dm;
    dm.header.frame_id = base_frame_;
    dm.header.stamp = ros::Time::now();
    dm.info.resolution = map_resolution_;
    dm.info.width = size_x;
    dm.info.height = size_y;
    dm.info.origin = fixed_origin_;
    // dm.data = data;
    dm.data.resize(size_x * size_y);
    std::transform(data.begin(), data.end(), dm.data.begin(), [](unsigned char c){
        return static_cast<signed char>(c);
    });
    data_map_pub_.publish(dm);
}

bool VelocityProcessor::worldToMap(double wx, double wy,
                                      unsigned int& mx, unsigned int& my){
    if (wx < fixed_origin_.position.x || wy < fixed_origin_.position.y)
        return false;
    mx = static_cast<unsigned int>((wx - fixed_origin_.position.x) / map_resolution_);
    my = static_cast<unsigned int>((wy - fixed_origin_.position.y) / map_resolution_);
    return mx < size_x && my < size_y;
}

void VelocityProcessor::mapToWorld(unsigned int mx, unsigned int my,
                                      double& wx, double& wy){
    wx = fixed_origin_.position.x + (mx + 0.5) * map_resolution_;
    wy = fixed_origin_.position.y + (my + 0.5) * map_resolution_;
}
} // namespace dmvm