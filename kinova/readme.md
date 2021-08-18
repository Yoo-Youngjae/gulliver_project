# Pick and Place Using Kinova Gen3 lite
## About The Project
This project is about Picking and Placing 6 objects with 3 classes. The robot used in this project is Kinova Gen3 lite. 
### procedure
1. Detect object using YOLOv3.
2. Caculate the coordinate of object by pointcloud. Device used is realsense 2.
3. The robot moves to the coordinate and pick detected object.
4. Place the object. Place location is differed by the classes of objects.

Connect Rosbridge: roslaunch rosbridge_server rosbridge_websocket.launch
Start kortex_driver: roslaunch kortex_driver kortex_driver.launch arm:=gen3_lite
Start realsense: roslaunch realsense2_camera rs_rgbd.launch enable_pointcloud2:=true
