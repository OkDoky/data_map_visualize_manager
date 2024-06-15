#ifndef POINTCLOUD2PROCESSOR_H
#define POINTCLOUD2PROCESSOR_H

#include <string>
#include <ros/ros.h>
#include <rxcpp/rx.hpp>
#include <algorithm>
#include <mutex>
#include <vector>
#include <unordered_set>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <processor_interface.h>

namespace dmvm {

class PointCloud2Processor : public ProcessorInterface {
public:
    PointCloud2Processor();
    void initialize(const std::string& ns) override;
    void start() override;
    void getCharMap(std::vector<unsigned char>& data_map) override;
    void getOccupiedIndices(std::vector<size_t>& occupied_indices) override;
    void getDataValue(unsigned char& data_value) const override;
private:
    void onPointCloud2Received(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void processData(const sensor_msgs::PointCloud2& msg);
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
    inline unsigned int getIndex(unsigned int mx, unsigned int my) const{
        return my * size_x + mx;
    }
    void publishDataMap(const std::vector<unsigned char>& data);

    std::string input_topic_;
    std::string output_topic_;
    std::string base_frame_;
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_, data_map_pub_;

    // for tf2 listener
    tf2_ros::Buffer tf2_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // data map using OccupancyGrid to visualize
    nav_msgs::OccupancyGrid pcl2_data_map_;
    std::unordered_set<unsigned int> index_flags_;

    // for DM
    std::vector<unsigned char> data_map_;
    std::vector<size_t> occupied_indices_;
    size_t data_map_size_;
    unsigned int size_x, size_y;
    unsigned char default_value_;
    float map_height_, map_width_, map_resolution_;
    float max_obstacle_height_;
    unsigned char data_value_;
    geometry_msgs::Pose fixed_origin_;

    // lock data map
    std::mutex data_map_mutex_;
    
    // for reactive program
    rxcpp::subjects::subject<sensor_msgs::PointCloud2> pointcloud2_subject_;
    rxcpp::observable<sensor_msgs::PointCloud2> pointcloud2_observable_;
};
} // namespace dmvm
#endif // POINTCLOUD2PROCESSOR_H