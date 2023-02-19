cmake_minimum_required(VERSION 2.8.3)
project(slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  nmea_msgs
  nav_msgs
  sensor_msgs
  message_generation
  cv_bridge
  message_filters
  cmake_modules #for eigen used in sensing/kalman_fusion
  pcl_ros
  tf
  tf2
  image_transport
)
#for eigen used in sensing/kalman_fusion
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
#

find_package(OpenCV 3.2.0 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules (gsl REQUIRED gsl)

add_message_files(
  DIRECTORY msg
  FILES 
  Old_Data.msg Data.msg Kinematics.msg Gps.msg Imu.msg Lidar.msg LidarPoint.msg Cluster.msg Pixel.msg GlobalPathPoint.msg 
  Yolomaster.msg Yoloinfo.msg Clustermaster.msg imgCluster.msg Clusters.msg ParkingSpot.msg Float_header.msg VehicleState.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs nmea_msgs nav_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS message_runtime DEPENDS Eigen)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

##add_executable for cone_clustering
add_executable(cone_clustering src/cone_clustering/cone_clustering.cpp)
target_link_libraries(cone_clustering ${catkin_LIBRARIES})
add_dependencies(cone_clustering slam_generate_messages_cpp)

add_executable(cone_decoder src/cone_clustering/cone_decoder.cpp)
target_link_libraries(cone_decoder ${catkin_LIBRARIES})
add_dependencies(cone_decoder slam_generate_messages_cpp)

add_executable(cone_costmap src/cone_clustering/cone_costmap.cpp)
target_link_libraries(cone_costmap ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(cone_costmap slam_generate_messages_cpp)

add_executable(cone_color src/cone_clustering/cone_color.cpp)
target_link_libraries(cone_color ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(cone_color slam_generate_messages_cpp)


##add_executable for costmap
add_executable(local_costmap_publisher src/costmap/local_costmap_publisher.cpp lib/XYToPixel.cpp)
target_link_libraries(local_costmap_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(local_costmap_publisher slam_generate_messages_cpp)

add_executable(local_path_publisher src/global_path/local_path_publisher.cpp lib/XYToPixel.cpp)
target_link_libraries(local_path_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(local_path_publisher slam_generate_messages_cpp)

add_executable(lane_costmap_publisher src/costmap/lane_costmap_publisher.cpp lib/XYToPixel.cpp)
target_link_libraries(lane_costmap_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(lane_costmap_publisher slam_generate_messages_cpp)

add_executable(lane_sector src/costmap/lane_sector.cpp lib/XYToPixel.cpp)
target_link_libraries(lane_sector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(lane_sector slam_generate_messages_cpp)

##add_executable for global path

add_executable(visualize_global_path src/global_path/visualize_global_path.cpp)
target_link_libraries(visualize_global_path ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(visualize_global_path slam_generate_messages_cpp)

add_executable(manipulate_global_path src/global_path/manipulate_global_path.cpp)
target_link_libraries(manipulate_global_path ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(manipulate_global_path slam_generate_messages_cpp)

add_executable(path_speed_adder src/global_path/path_speed_adder.cpp)
target_link_libraries(path_speed_adder ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(path_speed_adder slam_generate_messages_cpp)

##add_executable for obstacle_costmap
add_executable(obstacle_detector src/obstacle_costmap/obstacle_detector.cpp lib/XYToPixel.cpp)
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(obstacle_detector slam_generate_messages_cpp)

add_executable(obstacle_image src/obstacle_costmap/obstacle_image.cpp)
target_link_libraries(obstacle_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(obstacle_image slam_generate_messages_cpp)

add_executable(obstacle_costmap_publisher src/obstacle_costmap/obstacle_costmap_publisher.cpp)
target_link_libraries(obstacle_costmap_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(obstacle_costmap_publisher slam_generate_messages_cpp)

add_executable(decaying_costmap_publisher src/obstacle_costmap/decaying_costmap_publisher.cpp lib/XYToPixel.cpp)
target_link_libraries(decaying_costmap_publisher ${Eigen_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(decaying_costmap_publisher slam_generate_messages_cpp)

add_executable(parking_spot_detector src/obstacle_costmap/parking_spot_detector.cpp)
target_link_libraries(parking_spot_detector ${catkin_LIBRARIES})
add_dependencies(parking_spot_detector slam_generate_messages_cpp)

add_executable(imu_getter src/obstacle_costmap/imu_getter.cpp)
target_link_libraries(imu_getter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(imu_getter slam_generate_messages_cpp)

##add_executable for sensing
add_executable(gps_serial_decoder src/sensing/gps_serial_decoder.cpp lib/UTM.cpp)
target_link_libraries(gps_serial_decoder ${catkin_LIBRARIES})
add_dependencies(gps_serial_decoder slam_generate_messages_cpp)

add_executable(imu_decoder src/sensing/imu_decoder.cpp lib/UnixtimeToSec.cpp)
target_link_libraries(imu_decoder ${catkin_LIBRARIES})
add_dependencies(imu_decoder slam_generate_messages_cpp)

add_executable(lidar_decoder src/sensing/lidar_decoder.cpp)
target_link_libraries(lidar_decoder ${catkin_LIBRARIES})
add_dependencies(lidar_decoder slam_generate_messages_cpp)

add_executable(kalman_fusion src/sensing/kalman_fusion.cpp lib/XYToPixel.cpp)
target_link_libraries(kalman_fusion ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(kalman_fusion slam_generate_messages_cpp)

catkin_install_python(PROGRAMS 
  #src/sensing/lidar_decoder.py
  src/sensing/message_filter.py
  src/obstacle_costmap/cluster_to_pointcloud.py
  src/traffic_light/delivery_camera_publisher.py
  src/traffic_light/cone_camera_publisher.py
  src/traffic_light/customDetectVisualROS.py
  src/traffic_light/cone_box_publisher_newer.py
  src/delivery_sign/delivery_sign_publisher_new.py
  src/delivery_sign/delivery_sign_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
