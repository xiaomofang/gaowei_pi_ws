1.介绍
#该工程作用是进行SLAM建图、定位、路径规划,有两种建图方式，gmapping建图与cartographer建图。
在机器人端安装导航所需功能包：

安装 gmapping 包(用于构建地图):sudo apt install ros-<ROS版本>-gmapping

安装地图服务包(用于保存与读取地图):sudo apt install ros-<ROS版本>-map-server

安装 navigation 包(用于定位以及路径规划):sudo apt install ros-<ROS版本>-navigation


根据官方说明安装cartographer

2.建图
#2.1 gmapping建图
运行指令：
 roslaunch myrobot_navigation create_map_gmapping.launch
保存建图，打开新的终端，运行如下指令：
 rosrun map_server map_saver -f map_name

#2.2 cartographer建图
运行指令：
 roslaunch myrobot_navigation create_map_carto.launch
保存建图，打开新的终端，运行如下指令：
cd launch
./finish_slam_2d.sh

3.定位与导航
#3.1 使用gmapping地图定位与路径规划
运行指令:
 roslaunch myrobot_navigation gmap_navigation.launch

#3.2 使用cartographer地图定位与路径规划
 roslaunch myrobot_navigation carto_navigation_carto.launch



####
仿真环境下开启建图与运动规划
####
attion:
1.cartographer revo_lds.lua中几个坐标系配置
a) 在只使用激光雷达的时候（tracking_frame=”laser”, publish_frame=”laser”）

b) 使用里程计+激光雷达时（tracking_frame=”base_footprint”, publish_frame=”odom”）
同时，开启odom ，use_odometry = true,

c) 使用IMU+激光+里程计时（tracking_frame=”imu_link”, publish_frame=”odom”）
具体可参考链接:http://t.csdn.cn/7nWbK
注意tf树正确，是否提供odom的tf, 如果提供则tf树为map->odom->footprint，否则，tf树为map->footprint
2.仿真中， 对应定位lacunch文件，比如demo_revo_lds_localization.launch，中 <param name="/use_sim_time" value="true" />设为true，如果是实物小车则为fasle

3.仿真环境下建图与导航指令

a) 开启仿真环境
roslaunch urdf02_gazebo demo03_env.launch
b) 开启建图/导航launch文件，根据需求开启，exp:
roslaunch myrobot_navigation carto_navigation_carto.launch 

c) 补充，如果警告报错，可以屏蔽上面launch中Lidar driver，robot_description的xacro配置

####



