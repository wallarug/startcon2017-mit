# Setup after checking out
if the computer doesn't have the following packages installed you can
```
git clone https://github.com/ros-drivers/ackermann_msgs.git src/ackermann_msgs
git clone https://github.com/wjwwood/serial.git src/serial
git clone https://github.com/robopeak/rplidar_ros.git src/rplidar_ros
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git src/hector_slam
```
or
```
sudo apt-get install ros-kinetic-ackermann-msgs
sudo apt-get install ros-kinetic-serial
sudo apt-get install ros-kinetic-rplidar-ros    # need to install this for rplidar
sudo apt-get install ros-kinetic-hector-slam    # SLAM packages
```

# install other dependencies
```
sudo apt-get install ros-kinetic-vision-opencv
sudo apt-get install ros-kinetic-pointcloud-to-laserscan
```

To build and set environment variables
```
catkin_make
. devel/setup.sh
```


# To run the car
```
# run car + camera
roslaunch the_robotics_club mit-car.launch

# run car + camera + lidar + hectorslam
roslaunch the_robotics_club mit-car-lidar.launch
# then on a remote ubuntu machine run
roslaunch the_robotics_club mit-car-lidar-observer.launch

# run car + camera + cv
roslaunch the_robotics_club mit-car-vision.launch
```

# Using the Computer Vision launcher
The joystick is used as safety for now. 
Top left trigger: as is previously, used for manual control.
Top right trigger: activates driving based computer vision & PID


# To observe the camera remotely with rqt_gui
```
export ROS_MASTER_URI=http://x.x.x.x:11311 the_robotics_club observer.launch
```



# How to watch the GPU memory usage
looks like the Zed uses about 1850/7850MB just being in the desktop with a few terminals and gedit running
```
sudo ~/tegrastats
```


# Nodes and topics
![alt text](../images/vesc_nodes.png "vesc nodes")

![alt text](../images/zed_topics.png "zed topics")
