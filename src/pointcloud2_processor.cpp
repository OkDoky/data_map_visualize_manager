#include "pointcloud2_processor.h"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(dmvm::PointCloud2Processor, ProcessorInterface)


namespace dmvm {
PointCloud2Processor::PointCloud2Processor(){}

void PointCloud2Processor::initialize(const std::string& ns){
    ROS_INFO("[PointCloud2Processor] ns : %s", ns.c_str());
    private_nh_ = ros::NodeHandle(ns);
    nh_ = ros::NodeHandle("");
    ps_nh_ = ros::NodeHandle(ros::this_node::getName());
    private_nh_.param<std::string>("input_data_type", input_data_type_, "LaserScan");
    private_nh_.param<std::string>("input_pcl_topic", input_topic_, "input_pcl");
    private_nh_.param<std::string>("output_pcl_topic", output_topic_, "output_pcl");
    ps_nh_.param<std::string>("map_frame", base_frame_, "base_footprint");
    
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
    data_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("data_map", 1);

    tf2_listener_ = boost::make_shared<tf2_ros::TransformListener>(tf2_buffer_);

    private_nh_.param<float>("map_height", map_height_, 10.0);
    private_nh_.param<float>("map_width", map_width_, 10.0);
    private_nh_.param<float>("map_resolution", map_resolution_, 0.05);
    private_nh_.param<float>("max_obstacle_height", max_obstacle_height_, 1.5);
    int data_value_int;
    private_nh_.param<int>("data_value", data_value_int, 64);
    data_value_ = static_cast<unsigned char>(data_value_int);
    size_x = static_cast<unsigned int>(map_width_ / map_resolution_);
    size_y = static_cast<unsigned int>(map_height_ / map_resolution_);
    data_map_size_ = static_cast<size_t>(size_x * size_y);
    default_value_ = 0;
    data_map_.resize(data_map_size_, default_value_);

    fixed_origin_.position.x = -4.95;
    fixed_origin_.position.y = -4.95;
    fixed_origin_.orientation.w = 1.0;

    if (input_data_type_ == "PointCloud2") {
        subscriber_ = nh_.subscribe(input_topic_, 1, &PointCloud2Processor::onPointCloud2Received, this);
        publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        pointcloud2_observable_ = pointcloud2_subject_.get_observable();
        pointcloud2_observable_.subscribe([this](const sensor_msgs::PointCloud2& msg) {
            this->processData(msg);
        });
    } else if (input_data_type_ == "LaserScan") {
        subscriber_ = nh_.subscribe(input_topic_, 1, &PointCloud2Processor::onLaserScanReceived, this);
        laserscan_observable_ = laserscan_subject_.get_observable();
        laserscan_observable_.subscribe([this](const sensor_msgs::PointCloud2& msg) {
            this->processData(msg);
        });
    } else {
        
    }
}

void PointCloud2Processor::start(){
    ros::spin();
}

void PointCloud2Processor::onPointCloud2Received(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pointcloud2_subject_.get_subscriber().on_next(*msg);
}

void PointCloud2Processor::onLaserScanReceived(const sensor_msgs::LaserScan::ConstPtr& msg){
    sensor_msgs::PointCloud2 cloud;
    cloud.header = msg->header;
    try {
        projector_.projectLaser(*msg, cloud);
    } catch (tf2::TransformException &ex){
        ROS_WARN("[PointCloud2Processor] High Fidelity enabled, but TF returned a transform exception to frame %s : %s", base_frame_.c_str(), ex.what());
    }
    laserscan_subject_.get_subscriber().on_next(cloud);
}

void PointCloud2Processor::processData(const sensor_msgs::PointCloud2& msg){
    // transform sensor frame cloud -> base frame cloud
    sensor_msgs::PointCloud2 transformed_cloud;
    try{
        tf2_buffer_.transform(msg, transformed_cloud, base_frame_);
    } catch (tf2::TransformException &ex){
        ROS_WARN("%s", ex.what());
        return;
    }
    
    transformed_cloud.header.frame_id = base_frame_;

    // publish transformed cloud
    publisher_.publish(transformed_cloud);

    // Lock the mutex before updating data_map_
    std::lock_guard<std::mutex> lock(data_map_mutex_);

    // Resize and Reset data_map_
    std::fill(data_map_.begin(), data_map_.end(), default_value_);
    occupied_indices_.clear();
    index_flags_.clear();

    // Get the offsets for x, y, z fields
    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (const auto& field : transformed_cloud.fields) {
        if (field.name == "x") x_offset = field.offset;
        else if (field.name == "y") y_offset = field.offset;
        else if (field.name == "z") z_offset = field.offset;
    }

    if (x_offset == -1 || y_offset == -1 || z_offset == -1) {
        ROS_WARN("PointCloud2 message does not contain x, y, z fields");
        return;
    }

    // fill data map
    for (size_t i = 0; i < transformed_cloud.width * transformed_cloud.height; ++i) {
        float px = *reinterpret_cast<const float*>(&transformed_cloud.data[i * transformed_cloud.point_step + x_offset]);
        float py = *reinterpret_cast<const float*>(&transformed_cloud.data[i * transformed_cloud.point_step + y_offset]);
        float pz = *reinterpret_cast<const float*>(&transformed_cloud.data[i * transformed_cloud.point_step + z_offset]);

        if (pz > max_obstacle_height_){
            continue;
        }

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
    // publishDataMap(data_map_);
}

void PointCloud2Processor::getCharMap(std::vector<unsigned char>& data_map) {
    // Lock the mutex before accessing data_map_
    std::lock_guard<std::mutex> lock(data_map_mutex_);
    data_map = data_map_;
}

void PointCloud2Processor::getOccupiedIndices(std::vector<size_t>& occupied_indices) {
    std::lock_guard<std::mutex> lock(data_map_mutex_);
    occupied_indices = occupied_indices_;
}

void PointCloud2Processor::getDataValue(unsigned char& data_value) const{
  data_value = data_value_;
}

void PointCloud2Processor::publishDataMap(const std::vector<unsigned char>& data){
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

bool PointCloud2Processor::worldToMap(double wx, double wy,
                                      unsigned int& mx, unsigned int& my){
    if (wx < fixed_origin_.position.x || wy < fixed_origin_.position.y)
        return false;
    mx = static_cast<unsigned int>((wx - fixed_origin_.position.x) / map_resolution_);
    my = static_cast<unsigned int>((wy - fixed_origin_.position.y) / map_resolution_);
    return mx < size_x && my < size_y;
}

void PointCloud2Processor::mapToWorld(unsigned int mx, unsigned int my,
                                      double& wx, double& wy){
    wx = fixed_origin_.position.x + (mx + 0.5) * map_resolution_;
    wy = fixed_origin_.position.y + (my + 0.5) * map_resolution_;
}
} // namespace dmvm