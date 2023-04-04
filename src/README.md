1.介绍
##该工程作用是进行SLAM建图、定位、路径规划,有两种建图方式，gmapping建图与cartographer建图。
在机器人端安装导航所需功能包：

安装 gmapping 包(用于构建地图):sudo apt install ros-<ROS版本>-gmapping

安装地图服务包(用于保存与读取地图):sudo apt install ros-<ROS版本>-map-server

安装 navigation 包(用于定位以及路径规划):sudo apt install ros-<ROS版本>-navigation


根据官方说明安装cartographer

2.建图
##2.1 gmapping建图
运行指令：
 roslaunch myrobot_navigation create_map_gmapping.launch
保存建图，打开新的终端，运行如下指令：
 rosrun map_server map_saver -f map_name

##2.2 cartographer建图
运行指令：
 roslaunch myrobot_navigation create_map_carto.launch
保存建图，打开新的终端，运行如下指令：
 rosrun map_server map_saver -f map_name

3.定位与路径规划
##3.1 使用gmapping地图定位与路径规划
运行指令:
 roslaunch myrobot_navigation gmap_navigation.launch

##3.2 使用cartographer地图定位与路径规划
 roslaunch myrobot_navigation carto_navigation.launch





