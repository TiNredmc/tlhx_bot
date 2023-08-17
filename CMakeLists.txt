cmake_minimum_required(VERSION 3.8)
project(tlhx_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# apt package
find_package(ament_cmake REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(depthimage_to_laserscan REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
# build
# lidarslam_ros2
find_package(scanmatcher REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(lidarslam REQUIRED)
find_package(lidarslam_msgs REQUIRED)
find_package(graph_based_slam REQUIRED)
# openni2_camera
find_package(openni2_camera REQUIRED)


# Include all necessary directory to copy over to 'share' folder
install(
 DIRECTORY launch description params meshes
 DESTINATION share/${PROJECT_NAME}
)

ament_package()

