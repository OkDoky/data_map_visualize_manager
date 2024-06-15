#include "PointCloudProcessor.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_processor");
    dmvm::PointCloud2Processor processor("/r1/pcl", "/output_pointcloud_topic");
    processor.start();
    return 0;
}