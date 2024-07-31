#ifndef FOOTPRINTPROCESSOR_H
#define FOOTPRINTPROCESSOR_H

#include <string>
#include <ros/ros.h>
#include <algorithm>
#include <thread>
#include <atomic>
#include <math.h>
#include <mutex>
#include <iostream>
#include <vector>
#include <unordered_set>

#include <XmlRpcValue.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

#include <arr_parser.h>
#include <processor_interface.h>

struct MapLocation
{
  unsigned int x;
  unsigned int y;
};

inline double sign0(double x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

namespace dmvm {
class FootprintProcessor : public ProcessorInterface {
public:
    FootprintProcessor();
    void initialize(const std::string& ns) override;
    void start() override;
    void getCharMap(std::vector<unsigned char>& data_map) override;
    void getOccupiedIndices(std::vector<size_t>& occupied_indices) override;
    void getDataValue(unsigned char& data_value) const override;
    void getGoalValue(unsigned char& goal_value) const override{goal_value = 0;}
    void hasGoalValue(bool& has_goal) override{
        has_goal = false;
    }
    void startStandAlone();
private:
    std::string formatPolygon(const std::vector<geometry_msgs::Point>& polygon);
    void processData(const std::vector<geometry_msgs::Point>& polygon);
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
    
    // make footprint methods
    std::vector<geometry_msgs::Point> makeFootprintFromParams();
    bool makeFootprintFromString(
        const std::string& footprint_string, std::vector<geometry_msgs::Point>& footprint);
    std::vector<geometry_msgs::Point> makeFootprintFromXMLRPC(
        XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name);
    double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name);
    void writeFootprintToParam(const std::vector<geometry_msgs::Point>& footprint);
    inline unsigned int getIndex(unsigned int mx, unsigned int my) const{
        return my * size_x + mx;
    }
    void updateDataMap(nav_msgs::OccupancyGrid& dm);
    void publishDataMap(nav_msgs::OccupancyGrid& dm);
    void publishThread(const std::vector<unsigned char>& data_map);
    void interpolatedFootprint(const std::vector<geometry_msgs::Point>& footprint_edge,
                               std::vector<geometry_msgs::Point>& interpolated_footprint);
    void padFootprint(std::vector<geometry_msgs::Point>& footprint);

    std::string base_frame_;
    ros::NodeHandle nh_, private_nh_, ps_nh_;
    ros::Publisher data_map_pub_;

    // data map using OccupancyGrid to visualize
    nav_msgs::OccupancyGrid footprint_data_map_;
    std::unordered_set<unsigned int> index_flags_;

    // footprint padding
    bool use_padding_;
    float padding_size_;

    // lock
    std::mutex data_map_mutex_;

    // for DM
    std::vector<unsigned char> data_map_;
    std::vector<size_t> occupied_indices_;
    size_t data_map_size_;
    unsigned int size_x, size_y;
    unsigned char default_value_;
    float map_height_, map_width_, map_resolution_;
    unsigned char data_value_;
    geometry_msgs::Pose fixed_origin_;
    std::vector<geometry_msgs::Point> footprint_;

    // for thread
    std::thread publish_thread_;
    std::atomic<bool> running_;
    int update_rate_;
    float sleep_rate_;
};
} // namespace dmvm
#endif // FOOTPRINTPROCESSOR_H