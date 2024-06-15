#ifndef PROCESSORMANAGER_H
#define PROCESSORMANAGER_H

#include <ros/ros.h>
#include <pluginlib/class_loader.hpp>
#include <processor_interface.h>
#include <vector>
#include <memory>
#include <boost/shared_ptr.hpp>

class ProcessorManager {
    public:
        ProcessorManager();
        void initialize();
        void start();
        void publishCombinedMap();
        void updateCombinedMap();
    
    private:
        ros::NodeHandle nh_;
        ros::Publisher map_pub_;
        pluginlib::ClassLoader<ProcessorInterface> plugin_loader_;
        std::vector<boost::shared_ptr<ProcessorInterface>> processors_;
        std::vector<unsigned char> combined_data_map_;

        double map_width_, map_height_;
        double update_rate_;
        unsigned int size_x_, size_y_;
        unsigned char default_value_;
        double map_resolution_;
        std::string map_frame_;
};
#endif // PROCESSORMANAGER_H

