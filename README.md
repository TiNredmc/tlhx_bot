# tlhx_bot
My first ever ROS2 robot project

# Required external dependencies

Openni2_camera (on ros-drivers)
```
git clone https://github.com/E12-CO/openni2_camera.git
```

lidarslam_ros2 (by  rsasaki0109)
```
git clone --recursive https://github.com/TiNredmc/lidarslam_ros2
```
# Required internal dependencies

- depth_image_proc
- depthimage_to_laserscan
- robot_state_publisher
- joint_state_publisher
- joint_state_publisher_gui
- xacro
- ros-humble-libg2o

# Hardware Requirement (Tentative)
 - Orange Pi Zero 3 4GB
    - Ubuntu Jammy server image (download from orange pi website)
    - python SYSFS GPIO library [(udev rule here)](https://gist.github.com/TiNredmc/bbe6ad56a5a85d00a34b44c0830795c8) 
 - Asus Xtion (PS1080-E3IWBD001305)
