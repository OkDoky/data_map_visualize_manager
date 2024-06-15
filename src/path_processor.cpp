#include <chrono>
#include <iostream>
#include "path_processor.h"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(dmvm::PathProcessor, ProcessorInterface)

namespace dmvm {
PathProcessor::PathProcessor(){}

void PathProcessor::initialize(const std::string& ns){
    ROS_INFO("[PathProcessor] ns : %s", ns.c_str());
    private_nh_ = ros::NodeHandle(ns);
    nh_ = ros::NodeHandle("");
    private_nh_.param<std::string>("path_topic", path_topic_, "path");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_footprint");
    private_nh_.param<bool>("stand_alone", stand_alone_, false);
    private_nh_.param<double>("update_rate", update_rate_, 10.0);
    if (update_rate_ != 0){
        sleep_rate_ = static_cast<float>(1.0/update_rate_);
    } else {
        sleep_rate_ = 0.1;
    }
    subscriber_ = nh_.subscribe(path_topic_, 1, &PathProcessor::onPathReceived, this);
    if (stand_alone_){
        data_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("data_map", 1);
    }

    tf2_listener_ = boost::make_shared<tf2_ros::TransformListener>(tf2_buffer_);

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
    update_thread_ = std::thread(&PathProcessor::transformThread, this);
    // update_thread_.detach();
}

void PathProcessor::start(){
    ros::spin();
}

void PathProcessor::onPathReceived(const nav_msgs::Path::ConstPtr& msg){
    // path_subject_.get_subscriber().on_next(*msg);
    last_path_ = *msg;
}

void PathProcessor::transformThread(){
    int sleep_rate_millis_ = static_cast<int>(sleep_rate_ * 1000.0);
    while (ros::ok()){
        processData(last_path_);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_rate_millis_)); //
    }
}

void PathProcessor::processData(const nav_msgs::Path& last_path){
    // transform global frame path -> base frame cloud
    nav_msgs::Path transformed_path;
    if (!last_path.header.frame_id.empty() && last_path.header.frame_id != base_frame_){
        transformed_path.header = last_path.header;
        for (size_t i=0; i < last_path.poses.size(); i++){
            try{
                geometry_msgs::PoseStamped transformed_pose;
                tf2_buffer_.transform(last_path.poses[i], transformed_pose, base_frame_, ros::Duration(2.0));
                transformed_path.poses.push_back(transformed_pose);
            } catch (tf2::TransformException &ex){
                ROS_WARN("%s", ex.what());
                return;
            }
        }
    }else{
        transformed_path = last_path;
    }
    transformed_path.header.frame_id = base_frame_;
    
    // Lock the mutex before updating data_map_
    std::lock_guard<std::mutex> lock(data_map_mutex_);

    // Resize and Reset data_map_
    std::fill(data_map_.begin(), data_map_.end(), default_value_);
    occupied_indices_.clear();
    index_flags_.clear();

    // fill data map
    for (size_t i = 0; i < transformed_path.poses.size(); i++) {
        float px = transformed_path.poses[i].pose.position.x;
        float py = transformed_path.poses[i].pose.position.y;

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

void PathProcessor::getCharMap(std::vector<unsigned char>& data_map) {
    // Lock the mutex before accessing data_map_
    std::lock_guard<std::mutex> lock(data_map_mutex_);
    data_map = data_map_;
}

void PathProcessor::getOccupiedIndices(std::vector<size_t>& occupied_indices) {
    std::lock_guard<std::mutex> lock(data_map_mutex_);
    occupied_indices = occupied_indices_;
}

void PathProcessor::getDataValue(unsigned char& data_value) const{
  data_value = data_value_;
}

void PathProcessor::publishDataMap(const std::vector<unsigned char>& data){
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

bool PathProcessor::worldToMap(double wx, double wy,
                                      unsigned int& mx, unsigned int& my){
    if (wx < fixed_origin_.position.x || wy < fixed_origin_.position.y)
        return false;
    mx = static_cast<unsigned int>((wx - fixed_origin_.position.x) / map_resolution_);
    my = static_cast<unsigned int>((wy - fixed_origin_.position.y) / map_resolution_);
    return mx < size_x && my < size_y;
}

void PathProcessor::mapToWorld(unsigned int mx, unsigned int my,
                                      double& wx, double& wy){
    wx = fixed_origin_.position.x + (mx + 0.5) * map_resolution_;
    wy = fixed_origin_.position.y + (my + 0.5) * map_resolution_;
}
} // namespace dmvm