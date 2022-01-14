## 小车启动

ssh coo@10.42.0.1

密码： cooneo_coo

```shell
cd catkin_ws

roslaunch turn_on_wheeltec_robot base_camera.launch
```



## #离线采集数据

```shell
source devel/setup.bash

roslaunch rslidar_sdk start.launch         #  启动雷达

rosrun rs_to_velodyne rs_to_velodyne XYZIRT XYZIR			#  映射话题

cd /home/lory/lego_loam_ws/rosbag

rosbag record  /velodyne_points /imu_data  /usb_cam/image_raw/compressed  -O  cooneo_indoor.bag
```



## #建图

```shell
roslaunch lego_loam run.launch

rosbag play *.bag --clock --topic /velodyne_points /imu/data  /usb_cam   /image_raw/compressed
```

