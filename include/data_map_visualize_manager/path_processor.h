#ifndef PathProcessor_H
#define PathProcessor_H

#include <string>
#include <ros/ros.h>
#include <algorithm>
#include <mutex>
#include <thread>
#include <atomic>
#include <iostream>
#include <vector>
#include <unordered_set>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <processor_interface.h>

namespace dmvm {

class PathProcessor : public ProcessorInterface {
public:
    PathProcessor();
    void initialize(const std::string& ns) override;
    void start() override;
    void getCharMap(std::vector<unsigned char>& data_map) override;
    void getOccupiedIndices(std::vector<size_t>& occupied_indices) override;
    void getDataValue(unsigned char& data_value) const override;
private:
    void onPathReceived(const nav_msgs::Path::ConstPtr& msg);
    void processData(const nav_msgs::Path& last_path);
    void transformThread();
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
    inline unsigned int getIndex(unsigned int mx, unsigned int my) const{
        return my * size_x + mx;
    }
    void publishDataMap(const std::vector<unsigned char>& data);

    std::string path_topic_;
    std::string base_frame_;
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber subscriber_;
    ros::Publisher data_map_pub_;
    bool stand_alone_;
    std::unordered_set<unsigned int> index_flags_;

    // for tf2 listener
    tf2_ros::Buffer tf2_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // for DM
    std::vector<unsigned char> data_map_;
    std::vector<size_t> occupied_indices_;
    size_t data_map_size_;
    unsigned int size_x, size_y;
    unsigned char default_value_;
    float map_height_, map_width_, map_resolution_;
    unsigned char data_value_;
    geometry_msgs::Pose fixed_origin_;

    // lock data map
    std::mutex data_map_mutex_;

    // update thread
    std::thread update_thread_;
    std::atomic<bool> running_;
    nav_msgs::Path last_path_;
    double update_rate_, sleep_rate_;
};
} // namespace dmvm
#endif // PathProcessor_H