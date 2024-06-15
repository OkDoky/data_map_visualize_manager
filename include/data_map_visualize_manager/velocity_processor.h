#ifndef VelocityProcessor_H
#define VelocityProcessor_H

#include <string>
#include <ros/ros.h>
#include <rxcpp/rx.hpp>
#include <algorithm>
#include <mutex>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <math.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>

#include <processor_interface.h>

struct Point{
    float px;
    float py;
};

namespace dmvm {

class VelocityProcessor : public ProcessorInterface {
public:
    VelocityProcessor();
    void initialize(const std::string& ns) override;
    void start() override;
    void getCharMap(std::vector<unsigned char>& data_map) override;
    void getOccupiedIndices(std::vector<size_t>& occupied_indices) override;
    void getDataValue(unsigned char& data_value) const override;
private:
    void onVelocityReceived(const geometry_msgs::Twist::ConstPtr& msg);
    void processData(const geometry_msgs::Twist& vel);
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
    inline unsigned int getIndex(unsigned int mx, unsigned int my) const{
        return my * size_x + mx;
    }
    void publishDataMap(const std::vector<unsigned char>& data);

    std::string velocity_topic_;
    std::string base_frame_;
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber subscriber_;
    ros::Publisher data_map_pub_;
    bool stand_alone_;
    std::unordered_set<unsigned int> index_flags_;

    // for DM
    std::vector<unsigned char> data_map_;
    std::vector<size_t> occupied_indices_;
    size_t data_map_size_;
    unsigned int size_x, size_y;
    unsigned char default_value_;
    float map_height_, map_width_, map_resolution_;
    unsigned char data_value_;
    double arrow_scale_;
    geometry_msgs::Pose fixed_origin_;

    // lock data map
    std::mutex data_map_mutex_;

    // for reactive program
    rxcpp::subjects::subject<geometry_msgs::Twist> velocity_subject_;
    rxcpp::observable<geometry_msgs::Twist> velocity_observable_;
};
} // namespace dmvm
#endif // VelocityProcessor_H