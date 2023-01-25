# Pick and Place Using Kinova Gen3 lite
## About The Project
This project is about Picking and Placing 6 objects with 3 classes. The robot used in this project is Kinova Gen3 lite. 
### procedure
1. Detect object using YOLOv3.
2. Caculate the coordinate of object by pointcloud. Device used is realsense 2.
3. The robot moves to the coordinate and pick detected object.
4. Place the object. Place location is differed by the classes of objects.
## Getting Started
### 1. Installation
+ #### ROS
  + [Install ROS](http://wiki.ros.org/ROS/Installation)
  + [Install ROS Kortex](https://github.com/Kinovarobotics/ros_kortex)
  + [Install Realsense ROS](https://github.com/IntelRealSense/realsense-ros)
+ #### YOLO
  + [Install Nvidia Driver](https://www.nvidia.co.kr/Download/index.aspx?lang=kr)
    + If possible, install version 470.
  + [Install CUDA](https://developer.nvidia.com/cuda-10.2-download-archive)
  + [Install CUDNN](https://developer.nvidia.com/cudnn)
  + [Install YOLO](https://pjreddie.com/darknet/yolo/)
    + Should change GPU,OPENCV,CUDNN to 1 in the Makefile
### 2. Kinova Gen3 lite Settings
[Kinova Gen3 lite User Guide](https://www.kinovarobotics.com/en/resources/gen3-lite-technical-resources)
+ explained in page 45-52.
### 3. Launch
+ Start kortex_driver
  ```
  roslaunch kortex_driver kortex_driver.launch arm:=gen3_lite ip_address:=192.168.2.10
  ```
  + Should change ip_address to 192.168.1.10 if you connected the robot to your computer by USB.
+ Start realsense
  ```
  roslaunch realsense2_camera rs_rgbd.launch enable_pointcloud2:=true
  ```
## How To Use
1. Power the robot and realsense 2.
2. Run kortex_driver and realsense 2 by the launch command.
3. place the objects in the area where the robot can reach and the realsense 2 can detect.
4. Check whether the objects are being detected properly by running yolo.py.
5. Run gen3_lite_tidy_up.py
