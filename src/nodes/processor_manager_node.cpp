#include <ros/ros.h>
#include <processor_manager.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "processor_manager_node");
    ProcessorManager manager;
    manager.initialize();
    manager.start();
    return 0;
}