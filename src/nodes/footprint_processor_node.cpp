#include "footprint_processor.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "footprint_processor");
    dmvm::FootprintProcessor processor;
    processor.startStandAlone();
    return 0;
}