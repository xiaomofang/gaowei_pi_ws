#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include "my_car/Speed.h"

#define DISTANCE_THRESHOLD 0.15f
#define BASE_SPEED 10

class Car_Navigation
{

private:
    /* data */
    std::queue<Eigen::Vector2f> path;
    std::queue<Eigen::Vector2f> inflection_point;

    geometry_msgs::PoseStamped current_pos;
    my_car::Speed car_speed;
    int tracking_flag = 0;
    int path_size;
    int tracking_progress;
    int current_yaw, last_yaw, navi_yaw, yaw_break;



public:
    Car_Navigation(/* args */);
    ~Car_Navigation();

    void get_path(const std::vector<Eigen::Vector2i> load_path);
    void get_inflection_point(const std::vector<Eigen::Vector2i> load_path);
    void get_pose(const geometry_msgs::PoseStamped state);
    my_car::Speed get_speed();

    float speed_calculate(Eigen::Vector2f goal);
    bool is_tracking();
    void tracking(void);

};








#endif



