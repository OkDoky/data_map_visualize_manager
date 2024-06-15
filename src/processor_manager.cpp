#include <processor_manager.h>
#include <nav_msgs/OccupancyGrid.h>

ProcessorManager::ProcessorManager()
    : nh_("~"), default_value_(0), plugin_loader_("data_map_visualize_manager", "ProcessorInterface"){}

void ProcessorManager::initialize() {
    nh_.param<std::string>("map_frame", map_frame_, "base_footprint");
    nh_.param<double>("expected_update_rate", update_rate_, 10.0);
    nh_.param<double>("map_resolution", map_resolution_, 0.05);
    nh_.param<double>("map_width", map_width_, 10.0);
    nh_.param<double>("map_height", map_height_, 10.0);
    size_x_ = static_cast<unsigned int>(map_width_ / map_resolution_);
    size_y_ = static_cast<unsigned int>(map_height_ / map_resolution_);
    combined_data_map_.resize(size_x_ * size_y_, default_value_);

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("combined_data_map", 1);

    // load processors dynamically
    try{
        std::vector<std::string> processor_names;
        nh_.getParam("processors", processor_names);

        for (const auto& name : processor_names){
            std::string ns = "~/" + name;
            boost::shared_ptr<ProcessorInterface> processor = plugin_loader_.createInstance(name);
            processor->initialize(name);
            processors_.push_back(processor);
        }
    } catch (const pluginlib::PluginlibException& ex) {
        ROS_ERROR("[DMVM] Failed to load a processor. Error : %s", ex.what());
    }
}

void ProcessorManager::start(){
    ros::Rate rate(update_rate_); // 10hz
    while (ros::ok()){
        updateCombinedMap();
        publishCombinedMap();
        ros::spinOnce();
        rate.sleep();
    }
}

// void ProcessorManager::updateCombinedMap(){
//     std::fill(combined_data_map_.begin(), combined_data_map_.end(), 0);

//     std::vector<unsigned char> processor_data_map;
//     for (const auto& processor : processors_){
//         processor->getCharMap(processor_data_map);
//         for (size_t i = 0; i < combined_data_map_.size(); ++i){
//             combined_data_map_[i] += processor_data_map[i];
//         }
//     }
// }

void ProcessorManager::updateCombinedMap(){
    std::fill(combined_data_map_.begin(), combined_data_map_.end(), 0);

    std::vector<size_t> occupied_indices;
    for (const auto& processor: processors_){
        unsigned char data_value;
        processor->getDataValue(data_value);
        processor->getOccupiedIndices(occupied_indices);
        if (occupied_indices.size() == 0){
            continue;
        }
        for (const auto& index: occupied_indices){
            combined_data_map_[index] += data_value;
        }
    }
}

void ProcessorManager::publishCombinedMap(){
    nav_msgs::OccupancyGrid combined_map;
    combined_map.header.frame_id = map_frame_;
    combined_map.header.stamp = ros::Time::now();
    combined_map.info.resolution = map_resolution_;
    combined_map.info.width = size_x_;
    combined_map.info.height = size_y_;
    combined_map.info.origin.position.x = -4.95;
    combined_map.info.origin.position.y = -4.95;
    combined_map.info.origin.orientation.w = 1.0;

    // combined_map.data = combined_data_map_;
    // std::fill(combined_map.data.begin(), combined_map.data.end(), combined_data_map_);
    combined_map.data.resize(combined_data_map_.size());
    std::transform(combined_data_map_.begin(), combined_data_map_.end(), combined_map.data.begin(),
                   [](unsigned char c) { return static_cast<int8_t>(c); });
    map_pub_.publish(combined_map);
}
