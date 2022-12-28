# rosbag2pcd
### rosbag转pcd，将激光雷达获取到的rosbag转化为pcd格式的点云文件

## 使用说明

### 1. 修改点云转化文件 map_generation_node.cpp
- 修改rosbag的话题topic
- 修改pcd点云文件的保存路径
- 控制rosbag的订阅和保存频率
- 如有需要，根据雷达或点云信息，调整xyz的偏移以及角度信息

### 2. 编译运行
- ros编译  
    ```bash
    catkin_make
    ```
- 运行
	```bash
	# 1st terminal for ROS core
	roscore
	# 2nd terminal for decoding node
	./devel/lib/obstacle_detection/map_generate
	# 3rd terminal for ROSBAG playing
	rosbag play xxx.bag
	```