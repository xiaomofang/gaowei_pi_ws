#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>
#include <termios.h>
#include "Astar_searcher.h"
#include "Navigation.h"


using namespace std;

void path_vis_setting(void);

ros::Subscriber _map_sub, _goal_sub, _state_sub;
ros::Publisher _visual_pub, _marker_pub, _speed_pub; 

visualization_msgs::Marker path_line;
nav_msgs::GridCells path_vis;

AstarPathFinder * _astar_path_finder = new AstarPathFinder();
Car_Navigation * _car_navigation = new Car_Navigation();

void visualization(std::vector<Eigen::Vector2i> path) {

    path_vis.cells.clear();
    path_line.points.clear();
    
    for( auto ptr : path) {
          
        geometry_msgs::Point obstacle; 
           
        obstacle.x = ptr[0] * 0.05 - 0.025 - 2.38297;
        obstacle.y = ptr[1] * 0.05 - 0.025 - 6.43577;
        obstacle.z = 0;
        path_vis.cells.push_back(obstacle);

        geometry_msgs::Point p;
        p.x = obstacle.x;
        p.y = obstacle.y;
        p.z = obstacle.z; 
        path_line.points.push_back(p);
    }
    
}

void recMapCallBack(const nav_msgs::OccupancyGrid msg ) {   

    _astar_path_finder->InitGridMap(msg);
    
}

void recGoalPosCallBack(const geometry_msgs::PoseStampedConstPtr & goal){
    
    _astar_path_finder->resetUsedGrids();

    if(!_car_navigation->is_tracking()) {
        if(!_astar_path_finder->isFind()) {
            if(_astar_path_finder->get_goal(goal)) {

                _astar_path_finder->AstarGraphSearch();
                _astar_path_finder->path = _astar_path_finder->get_path();
                // _car_navigation->get_path(_astar_path_finder->path);
                _car_navigation->get_inflection_point(_astar_path_finder->path);
                visualization(_astar_path_finder->path);

            }
        }
    }
    else {

        ROS_WARN("find path and tracking.....");    
    }
}

void recStatePosCallBack(const geometry_msgs::PoseStamped & pos) {

    _astar_path_finder->get_state(pos);
    _car_navigation->get_pose(pos);

}

int main(int argc, char ** argv) {

    ros::init(argc, argv, "path_plan");

    // ros::NodeHandle nh("~");
        ros::NodeHandle nh;

    _map_sub   = nh.subscribe("/map", 1, recMapCallBack);  //get map
    _goal_sub  = nh.subscribe("/move_base_simple/goal", 1, recGoalPosCallBack); //2d nav goal
    _state_sub = nh.subscribe("/tracked_pose",1, recStatePosCallBack); //订阅目标位置

    _marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    _visual_pub = nh.advertise<nav_msgs::GridCells>("/path", 1);
    //_speed_pub  = nh.advertise<my_car::Speed>("/car_speed",4);
    _speed_pub = nh.advertise<geometry_msgs::Twist>("/my_cmd_vel", 50);
    my_car::Speed send_speed;
    path_vis_setting();
    geometry_msgs::Twist  cmd_vel;
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    ros::Rate loop_rate(50);
   
    while(ros::ok())
    {

        if(_car_navigation->is_tracking()) {
            _car_navigation->tracking();
            send_speed = _car_navigation->get_speed();
        }
        else {
            send_speed.speed_0 = 0;
            send_speed.speed_1 = 0;
            send_speed.speed_2 = 0;
            send_speed.speed_3 = 0;
        }
        linear.x = (send_speed.speed_0+send_speed.speed_3)/2;
        linear.y = 0.0;
        linear.z = 0.0;
        angular.x = 0.0;
        angular.y = 0.0;
        angular.z = ((send_speed.speed_3-send_speed.speed_0)/(2*36));
        cmd_vel.linear=linear;
        cmd_vel.angular=angular;     
        _speed_pub.publish(cmd_vel);
        _visual_pub.publish(path_vis);
        _marker_pub.publish(path_line);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}

void path_vis_setting(void) {

    path_vis.header.frame_id = "map";
    path_vis.cell_height=0.05;
    path_vis.cell_width=0.05;
    path_vis.cells.resize(3);

    path_line.header.frame_id = "map";
    path_line.ns = "path_lines";
    path_line.action = visualization_msgs::Marker::ADD;
    path_line.pose.orientation.w = 1.0;
    path_line.id = 1;
    path_line.type = visualization_msgs::Marker::LINE_STRIP;
    path_line.scale.x = 0.01;
    path_line.color.b = 1.0;
    path_line.color.a = 1.0;

}
