# 3D-SLAM自搭平台   主动阿克曼 + RS16 + LPMS_IMU  LEGO_LOAM 建图

## 效果展示：

**详细建图(室内+室外)视频链接**：

[3D-SLAM自搭平台 主动阿克曼 + RS16 + LPMS_IMU LEGO_LOAM 室内建图](https://www.bilibili.com/video/BV1dL411c7u9/)

[3D-SLAM自搭平台 主动阿克曼 + RS16 + LPMS_IMU LEGO_LOAM 室外建图](https://www.bilibili.com/video/BV1mL4y1b7Q4/)

**实验的完整代码下载链接**：[https://github.com/kahowang/3D-SLAM-Multiple-robot-platforms/tree/main/Active-Ackerman-Slam/%E4%B8%BB%E5%8A%A8%E9%98%BF%E5%85%8B%E6%9B%BC%2BRS16%2BLPMS_IMU%20%20LEGO_LOAM%20%E5%BB%BA%E5%9B%BE](https://github.com/kahowang/3D-SLAM-Multiple-robot-platforms/tree/main/Active-Ackerman-Slam/%E4%B8%BB%E5%8A%A8%E9%98%BF%E5%85%8B%E6%9B%BC%2BRS16%2BLPMS_IMU%20%20LEGO_LOAM%20%E5%BB%BA%E5%9B%BE)

**录制的数据集下载链接**：https://pan.baidu.com/s/1IIGquusHYk_UmGeClXG9tA  密码: fwo3

<p align ="center">
<img src = "../../pic/car.gif "  alt ="car" width = 40%  height =30%; "/>
​				       <p align="center">自搭 3D SLAM 数据采集轮式移动平台</p>

<p align ="center">
<img src = "../../pic/lego-loam_indoor.gif "  alt ="lego-loam_indoor" width = 80%  height =80%; "/>
​               <p align="center">LEGO-LOAM 室内建图   </p>

<p align ="center">
<img src = "../../pic/lego-loam_outdoor.gif "  alt ="lego-loam_outdoor" width = 80%  height =80%; "/>                                                                                       
​               <p align="center">LEGO-LOAM 室外建图    base:深圳云之园</p>

## 前言：

本次实验为，使用自己搭建的移动底盘以及传感器，进行lego_loam 3D-SLAM建图，主要分为三大部分

```shell
a. LEGO_LOAM 论文概述摘要。(致敬作者Tixiao Shan  and Brendan Englot，及网上一系列的开源解释博主)
b.自搭平台，进行数据集采集，幷构建地图流程。(供参考)   
    现有设备：Robosense16 激光雷达，LPMS IMU,  自搭主动差速阿克曼底盘
c.数据集下载使用，搭建的平台，采集了室内、室外两个数据集，供大家尝试使用。（包含 雷达点云话题、IMU话题、相机话题）
```

##  　　　　　　　　　　　　　　　　　　　　　 　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　1.LEGO_LOAM 概述

### 1.1 论文摘要

​     我们提出了LeGO-LOAM，它是一种轻量级和地面优化的激光雷达里程计和建图方法，用于实时估计地面车辆的六自由度姿态。LeGO-LOAM是轻量级的，因为它可以在低功耗嵌入式系统上实现实时姿态估计。LeGO-LOAM经过地面优化，因为它在分割和优化步骤中利用了地面的约束。我们首先应用点云分割来滤除噪声，并进行特征提取，以获得独特的平面和边缘特征。然后，采用两步Levenberg-Marquardt优化方法，使用平面和边缘特征来解决连续扫描中六个自由度变换的不同分量。我们使用地面车辆从可变地形环境中收集的数据集，比较LeGO-LOAM与最先进的LOAM方法的性能，结果表明LeGO-LOAM在减少计算开销的情况下实现了相似或更好的精度。为了消除由漂移引起的姿态估计误差，我们还将LeGO-LOAM集成到SLAM框架中，并用KITTI数据集进行了测试。

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A1120210526113411804.png" width =60%  height =60%;" /></p>     

​	论文里面主要是和LOAM对比，其相比LOAM具有以下五个特点

### 1.2 系统性能

论文里面主要是和LOAM对比，其相比LOAM具有以下五个特点

a.轻量级，能在嵌入式设备上实时运行

<p align="center">

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A1120210526113615296.png"width =80%  height = 80%;" /></p>

b.地面优化，在点云处理部分加入了**分割模块**，这样做能够**去除地面点的干扰**，只在聚类的目标中提取特征。其中主要包括一个地面的提取（并没有假设地面是平面），和一个点云的分割，**使用筛选过后的点云再来提取特征点**，这样会大大提高效率。(如下图所示: 为在特征提取之前执行地面分割操作)

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A1120210526113505907.png" alt="20210526113505907" width =80%  height = 80%;" /></p>

c. 在提取特征点时，将**点云分成小块，分别提取特征点**，以保证特征点的均匀分布。在特征点匹配的时候，使用预处理得到的**segmenation标签筛选(点云分割为边缘点类和平面点类两类)**，再次提高效率。

d.两步L-M优化法估计**6个维度的里程计位姿**。先使用平面点优化高度，同时利用地面的方向**优化两个角度信息**。地面提取的平面特征用于在第一步中获得t_z,theta_roll,theta_pitch；再使用边角点优化剩下的**三个变量**，通过匹配从分段点云提取的边缘特征来获得其余部分变换t_x,t_y,theta_yaw。匹配方式还是scan2scan，以这种方式分别优化，效率提升40%，但是并没有造成精度的损失。

e.集成了[回环检测](https://blog.csdn.net/lzy6041/article/details/107670202)以校正运动估计漂移的能力（即使用gtsam作回环检测并作图优化，但是本质上仍然是基于欧式距离的回环检测，不存在全局描述子）。

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A1120210526113342924.png" alt="20210526113342924" width =80%  height = 80%" /></p>

## 2.自搭平台复现LEGO_LOAM建图

### 2.1 环境配置

#### 2.1.1 硬件配置

##### 2.1.1.1  速腾16线激光雷达

激光雷达通过网口进行配置，需要配置主从机网络。本机(机器人上的电脑或嵌入式设备)需要和雷达在同一个网段下。

```shell
雷达默认的出厂IP为 master_address :  192.168.1.200
本机设置IP为 slaver_address  : 192.168.1.102
```

如下图所示为速腾16线激光雷达的硬件驱动，将网线一头插在雷达硬件驱动，另一头插在电脑网口处。

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11rs.jpeg" alt="rs"width =50%  height = 50%;" /></p>

随后点击ubuntu右上角的有线连接，点击有线设置

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-01%2013-26-28%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-01 13-26-28 的屏幕截图" width =50%  height = 50%;" /></p>

点击有线连接右上角上的"+"

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-01%2013-35-56%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-01 13-35-56 的屏幕截图" width =50%  height = 50%;" /></p>

修改名称为自己的任意名称(这里修改为"Robosense16")

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-01%2013-36-16%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-01 13-36-16 的屏幕截图" width =50%  height = 50%;" /></p>

点击IPv4 ,填入地址 192.168.1.102 ,  子网掩码为 : 255.255.255.0

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-01%2013-36-34%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-01 13-36-34 的屏幕截图" width =50%  height = 50%;" /></p>

点击右上角的添加后，即可看到和雷达的有线连接成功

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-01%2013-36-40%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-01 13-36-40 的屏幕截图" width =50%  height = 50%;" /></p>

 #打开一个新的 terminal, 尝试是否能ping同雷达数据，如果接受到如下所示的数据，即配置雷达网口成功

```shell
ping 192.168.1.200   
```

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-01%2013-40-08%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-01 13-40-08 的屏幕截图" width =50%  height = 50%;" /></p>

##### 2.1.1.2  IMU 

使用的IMU为LPMS_URS2  9-axis 

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11lpms.jpeg" alt="lpms" width =50%  height = 50%;" /></p>

##### 2.1.1.3  主动差速阿克曼底盘

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11car.jpg" alt="car"width =50%  height = 50%;" /></p>

#### 2.1.2 软件配置

##### 2.1.2.1 速腾16线激光雷达软件ROS驱动下载

```shell
cd ~/catkin_ws/src
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
```

修改 rslidar_sdk CMakeLists.txt中的 配置文件

```shell
cd rslidar_sdk
sudo  gedit CMakeLists.txt
#将 set(POINT_TYPE XYZIRT)  修改为
set(COMPILE_METHOD CATKIN)
#将 set(POINT_TYPE XYZI)   修改为
set(POINT_TYPE XYZIRT)
```

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-01%2017-25-57%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-01 17-25-57 的屏幕截图" width =60%  height = 60%;" /></p>

修改 rslidar_sdk package_ros1.xml 改名为 package.xml

![2022-01-01 17-25-37 的屏幕截图](https://img-blog.csdnimg.cn/img_convert/f28a178b00eda2e6c838b7817e1da788.png)

修改  rslidar_sdk/config/comfig.yaml  第16行的 lidar_type为 ： RS16

```shell
      lidar_type: RS16            #LiDAR type - RS16, RS32, RSBP, RS128, RS80, RSM1, RSHELIOS
```

编译 

```shell
cd ~/catkin_ws
catkin_make 
```

##### 2.1.2.2 LPMS IMU 配置

```shell
catkin_make -DCMAKE_C_COMPILER=gcc-7 -DCMAKE_CXX_COMPILER=g++-7      #升级cmake版本
apt install ros-melodic-openzen-sensor     #下载 openzen-sensor
cd   lego_loam_ws/src
git clone  --recurse-submodules https://bitbucket.org/lpresearch/openzenros.git
cd ..   
catkin_make
```

##### 2.1.2.3 lego_loam 配置

依赖：

a. [ROS](http://wiki.ros.org/ROS/Installation) (tested with indigo, kinetic, and melodic)

b.[gtsam](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library, 4.0.0-alpha2)

安装 VTK

```shell
sudo apt install python-vtk
```

安装 gtsam：

```shell
wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip     # 安装gtsam
cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
cd ~/Downloads/gtsam-4.0.0-alpha2/
mkdir build && cd build
cmake ..
sudo make install
```

编译：

```shell
cd ~/catkin_ws/src					# 进入所在的工作空间
git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git       # git clone lego-loam 源码
cd ..
catkin_make -j1						   #  当第一次编译代码时，需要在“catkin_make”后面添加“-j1”以生成一些消息类型。将来的编译不需要“-j1”
```

##### 2.1.2.4  RS16 转 VLP16

由于lego_loam中默认接受的雷达话题为  /velodyne_points  ，接收的雷达点云信息为XYZIR ,而速腾的激光雷达驱动中只支持 XYZI 、XYZIRT 数据类型，所以需要对数据进行转换，可使用rs_to_velodyne  这个节点进行雷达点云消息格式的转换。

```shell
cd  ~/catkin_ws/src
git clone https://github.com/HViktorTsoi/rs_to_velodyne.git
catkin_make 
```

### 2.2 运行

#### 2.2.1 离线建图

##### 2.2.1.1  离线采集数据集

```shell
#运行小车底盘
ssh coo@10.42.0.1
password： cooneo_coo
cd catkin_ws
roslaunch turn_on_wheeltec_robot base_camera.launch
```

```shell
#采集数据
cd catkin_ws
source devel/setup.bash
roslaunch rslidar_sdk start.launch    # 启动RS16 雷达
```

```shell
rosrun rs_to_velodyne rs_to_velodyne XYZIRT XYZIR   # 启动雷达数据类型转换节点
```

```shell
rosrun openzen_sensor openzen_sensor_node      #  运行 LPMS  IMU
```

```shell
mkdir catkin_ws/rosbag
cd catkin_ws/rosbag
rosbag record  -a  -O lego_loam.bag										#  记录所有话题
```

##### 2.2.1.2 离线建图

```shell
roslaunch lego_loam run.launch										#   运行lego_loam
rosbag play *.bag --clock --topic /velodyne_points  /imu/data    # 选择使用 雷达点云话题和 IMU话题(若IMU配置一般，可只使用点云话题)
```

#### 2.2.2 在线建图

修改  run.launch 文件中   /use_sim_time 

```shell
<param name="/use_sim_time" value="false" />    # 在线建图设为"false",离线建图设为 "true"   
```

```shell
#运行小车底盘
ssh coo@10.42.0.1
password： cooneo_coo
cd catkin_ws
roslaunch turn_on_wheeltec_robot base_camera.launch
```

```shell
#打开雷达
cd catkin_ws
source devel/setup.bash
roslaunch rslidar_sdk start.launch    # 启动RS16 雷达
```

```shell
roslaunch lego_loam run.launch									#   运行lego_loam
```

### 2.3 最终效果

室内，室外数据集均已上传到网盘，供大家下载使用  

```shell
链接: https://pan.baidu.com/s/1IIGquusHYk_UmGeClXG9tA  
密码: fwo3
```

#### 2.3.1  室内离线建图

```shell
cd lego_loam_ws     #进入到，下载lego_loam 的工作空间中
roslaunch lego_loam run.launch        # 运行lego_loam  建图

cd rosbag               #进入到存储数据集的目录下
rosbag  play  cooneo_indoor_urs2.bag --clock --topic /velodyne_points /imu/data  /usb_cam/image_raw/compressed    # 运行室内数据集
```

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-10%2013-24-59%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-10 13-24-59 的屏幕截图"  width =60%  height = 60%;" />
​                                 <p align="center"> LEGO-LOAM 3D  室内3D地图 </p>

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-10%2013-25-20%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-10 13-25-20 的屏幕截图" width =60%  height = 60%;" />

​                               <p align="center"> LEGO-LOAM 3D  室内3D地图 </p>

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112d_gmapping.png" alt="2d_gmapping" width =60%  height = 60%;" />
​	                                  <p align="center">  2D  gmpping  建图</p>

#### 2.3.2 室外离线建图

```shell
cd lego_loam_ws     #进入到，下载lego_loam 的工作空间中
roslaunch lego_loam run.launch        # 运行lego_loam  建图

cd rosbag               #进入到存储数据集的目录下
rosbag  play  cooneo_outdoor_urs2.bag --clock --topic /velodyne_points /imu/data  /usb_cam/image_raw/compressed  # 运行室外数据集
```

<p align ="center">
<img src = "../../pic/lego-loam_outdoor2.gif "  alt ="lego-loam_outdoor" width = 80%  height =80%; "/>                                                                                       
​               <p align="center">LEGO-LOAM 室外建图    base:深圳云之园</p>

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-10%2013-13-45%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-10 13-13-45 的屏幕截图" width =80%  height = 80%;" /></p>

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-10%2013-14-05%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-10 13-14-05 的屏幕截图"width =80%  height = 80%;" /></p>

#### 2.3.3 保存地图

运行 run.launch  lego_loam 建图程序后，在rviz中点击勾选 **Map Cloud**，待程序结束后，CRTL + C 即可结束进程，会自动保存地图。

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11image-20220110195745488.png" alt="image-20220110195745488"width =35%  height = 35%;" /></p>

## 3. 数据及下载及使用

数据集下载地址 ： 链接: https://pan.baidu.com/s/1IIGquusHYk_UmGeClXG9tA  密码: fwo3

场景： 室内 、 室外

设备：二轮雅克曼底盘 、Robosense16线激光雷达、LPMS IMU

### 3.1 环境配置 

上述 **"2.1.2.3  lego_loam 配置"** 已说明 

### 3.2  修改相关配置及保存地图

#### 3.2.1 修改配置文件

```shell
sudo gedit  lego_loam_ws/src_Lego-LOAM-master/LeGo-LOAM/include/utility.h
```

**imuTopic**  修改为自己的imu_topic  ,  本次数据集的imu topic 为 "/imu/data" 

**fileDirectory**   中填写自己要保存的地图的绝对路径

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11termianal.png" alt="termianal" width =80%  height = 80%;" /></p>

#### 3.2.2 保存地图

运行 run.launch  lego_loam 建图程序后，在rviz中点击勾选**Map Cloud**，待程序结束后，CRTL + C 即可结束进程，建图成功。

<p align="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11image-20220110195745488.png" alt="image-20220110195745488"width =35%  height = 35%;" /></p>

### 3.3 构建地图

```shell
roslaunch lego_loam run.launch										#   运行lego_loam
```

```shell
#室内数据集
rosbag play cooneo_indoor_urs2.bag --clock --topic /velodyne_points  /imu/data     # 选择使用 雷达点云话题和 IMU话题(若IMU配置一般，可只使用点云话题)
#室外数据集
rosbag play cooneo_outdoor_urs2.bag --clock --topic /velodyne_points  /imu/data     # 选择使用 雷达点云话题和 IMU话题(若IMU配置一般，可只使用点云话题)
```

## 参考文献

[1] [LeGO-LOAM初探：原理，安装和测试](https://blog.csdn.net/learning_tortosie/article/details/86527542)

[[2]自动驾驶系列（三）速腾16线激光雷达驱动安装](https://blog.csdn.net/yongbutuifei/article/details/120038836)

[[3]激光SLAM框架学习之LeGO-LOAM框架---速腾Robosense-16线雷达室外建图和其他框架对比、录包和保存数据](https://blog.csdn.net/yongbutuifei/article/details/120038836)

[[4]SC-LEGO-LOAM 扩展以及深度解析（一）](https://www.guyuehome.com/34082)

[[5]LIO-SAM运行自己数据包遇到的问题解决--SLAM不学无数术小问题](https://blog.csdn.net/weixin_42141088/article/details/118000544#commentBox)　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　

[[6] SLAM学习笔记（二十）LIO-SAM流程及代码详解（最全）](https://blog.csdn.net/zkk9527/article/details/117957067)

[[7]LeGO-LOAM和LOAM的区别与联系](https://zhuanlan.zhihu.com/p/115986186)
    																																																																					                                                                                                                                    edited  by kaho  2022.1.10
