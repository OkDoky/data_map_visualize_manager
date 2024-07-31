#ifndef PROCESSORINTERFACE_H
#define PROCESSORINTERFACE_H

#include <ros/ros.h>
#include <vector>
#include <mutex>
#include <nav_msgs/OccupancyGrid.h>

class ProcessorInterface {
public:
    virtual ~ProcessorInterface() = default;
    virtual void initialize(const std::string& ns) = 0;
    virtual void start() = 0;
    virtual void getCharMap(std::vector<unsigned char>& data_map) = 0;
    virtual void getOccupiedIndices(std::vector<size_t>& occupied_indices) = 0;
    virtual void getDataValue(unsigned char& data_value) const = 0;
    virtual void getGoalValue(unsigned char& goal_value) const = 0;
    virtual void hasGoalValue(bool& has_goal) = 0;
private:

};

#endif // PROCESSORINTERFACE_H

