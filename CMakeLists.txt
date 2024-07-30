cmake_minimum_required(VERSION 3.0.2)
project(data_map_visualize_manager)

include_directories(
  /root/RxCpp/Rx/v2/src
)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  pluginlib
  laser_geometry
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include/data_map_visualize_manager
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DataMap.cfg
# )

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES dmvm
  CATKIN_DEPENDS 
    dynamic_reconfigure 
    geometry_msgs 
    nav_msgs 
    roscpp 
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    pluginlib
    laser_geometry
  DEPENDS
    EIGEN3
    Boost
)

add_library(dmvm
  src/footprint_processor.cpp
  src/pointcloud2_processor.cpp
  src/path_processor.cpp
  src/velocity_processor.cpp
)

add_dependencies(dmvm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dmvm ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(processor_manager_node src/processor_manager.cpp src/nodes/processor_manager_node.cpp)
target_link_libraries(processor_manager_node ${catkin_LIBRARIES} dmvm)

# add_executable(point_cloud_processor src/nodes/point_cloud_processor_node.cpp src/pointcloud2_processor.cpp)
# target_link_libraries(point_cloud_processor ${catkin_LIBRARIES})

# add_executable(footprint_processor src/nodes/footprint_processor_node.cpp src/footprint_processor.cpp src/arr_parser.cpp)
# target_link_libraries(footprint_processor ${catkin_LIBRARIES})

# install(TARGETS ${project}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

install(TARGETS dmvm processor_manager_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)