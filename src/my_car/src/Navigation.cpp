#include "Navigation.h"
#include "Serial_Send.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
Car_Navigation::Car_Navigation(/* args */)
{

}

Car_Navigation::~Car_Navigation()
{

}

bool Car_Navigation::is_tracking() {

    if(tracking_flag)
        return true;
    else 
        return false;
}

void Car_Navigation::get_inflection_point(const std::vector<Eigen::Vector2i> load_path) {

    while (!inflection_point.empty()) 
        inflection_point.pop();

    Eigen::Vector2i last_point(-500,-500);
    int error[2] = {-500, -500};
    int size = load_path.size();
    int n = 0;

    for(auto p: load_path) {

        if(last_point(0) == -500 && last_point(1) == -500) {
        }
        else if(error[0] == -500 && error[1] == -500){
            
            error[0] = p(0) - last_point(0);
            error[1] = p(1) - last_point(1);
        }
        else {
            if((p(0) - last_point[0]) == error[0] && (p(1) - last_point[1]) == error[1]) {

            }
            else {
                error[0] = p(0) - last_point(0);
                error[1] = p(1) - last_point(1);
                Eigen::Vector2f pose;
                pose(0) = p[0] * 0.05 - 0.025 - 5.46916;
                pose(1) = p[1] * 0.05 - 0.025 -4.84707;
                std::cout << "pose: x = " << pose(0) << " y = " << pose(1) <<std::endl;
                inflection_point.push(pose);
            }
        }

        n++;
        if(n == size) {

            Eigen::Vector2f pose;
            pose(0) = p[0] * 0.05 - 0.025 - 5.46916;
            pose(1) = p[1] * 0.05 - 0.025 -4.84707;
            std::cout << "pose: x = " << pose(0) << " y = " << pose(1) <<std::endl;
            inflection_point.push(pose);
        }

        last_point = p;
    }
    tracking_flag = 1;
    path_size = inflection_point.size();
    ROS_INFO("path_size = %d", path_size);

}

void Car_Navigation::get_path(const std::vector<Eigen::Vector2i> load_path) {
    while (!path.empty()) 
        path.pop();

    tracking_progress = 0;
    for(auto p: load_path) {
        Eigen::Vector2f pose;
        pose(0) = p[0] * 0.05 - 0.025 - 5.46916;
        pose(1) = p[1] * 0.05 - 0.025 -4.84707;
        std::cout << "pose: x = " << pose(0) << " y = " << pose(1) <<std::endl;
        path.push(pose);
    }
    tracking_flag = 1;
}

void Car_Navigation::get_pose(const geometry_msgs::PoseStamped state) {

    current_pos = state;
    Eigen::Quaterniond current_q(current_pos.pose.orientation.w, current_pos.pose.orientation.x, current_pos.pose.orientation.y, current_pos.pose.orientation.z);
    Eigen::Vector3d Current_Angle_Radian= current_q.matrix().eulerAngles(2,1,0);
    current_yaw = Current_Angle_Radian(0) * 180 / M_PI;
    if(std::abs(current_yaw - last_yaw) > 45) {
        if(yaw_break) 
            yaw_break = 0;
        else 
            yaw_break = 1;
    }
    if(yaw_break) {
        navi_yaw = current_yaw - 180;
    }
    else {
        navi_yaw = current_yaw ;
    }
    last_yaw = current_yaw;
}

// geometry_msgs::Twist  Car_speed;
geometry_msgs::Twist Car_Navigation::get_speed() {
    return Car_speed;
}
float Car_Navigation::speed_calculate(Eigen::Vector2f goal) {

    float   linear_speed=0.05 ;
    float   angular_speed=0.05;
    Car_speed.linear.y=0.0;
    Car_speed.linear.z=0.0;
    Car_speed.angular.x=0.0;
    Car_speed.angular.y=0.0;
    float dx = goal(0) - current_pos.pose.position.x;
    float dy = goal(1) - current_pos.pose.position.y;
   
    int point_angle = std::atan2(dy, dx) * 180 / M_PI; 
    int error_angle = 0;

    if((navi_yaw >= 0 && point_angle >= 0) || (navi_yaw < 0 && point_angle < 0)) {
        error_angle = std::abs(navi_yaw - point_angle);
    }
    else {
        
        error_angle = std::abs(navi_yaw - point_angle);
        int temp_angle = 360 - error_angle;
        error_angle = std::min(temp_angle, error_angle);
    } 

    float distance = std::sqrt((std::pow(dx,2.0) + std::pow(dy,2.0)));

    // std::cout << "goal_pose: x = " << goal(0) << " y = " << goal(1) <<std::endl;
    // std::cout << "current_pos: x = " << current_pos.pose.position.x <<  " y = " << current_pos.pose.position.y << std::endl;
    // std::cout << "C: " << navi_yaw << " p: " << point_angle << " error: " << error_angle <<  " dis: " << distance <<std::endl;
    
    int abs_error_angle = std::abs(error_angle);

    if(abs_error_angle >= 45) {
        Car_speed.linear.x=linear_speed/2;
        Car_speed.angular.z=2*angular_speed;
    }
    else if(abs_error_angle < 45 && abs_error_angle >= 10){

        if(point_angle <= 135 && point_angle >= -135) {
            if(navi_yaw >= point_angle) {
                Car_speed.linear.x=linear_speed;
                Car_speed.angular.z=angular_speed;
            }
            else {
                Car_speed.linear.x=linear_speed;
                Car_speed.angular.z=-angular_speed;
            }
        }
        else if(point_angle > 135 && point_angle <= 180) {
            
            if(navi_yaw >= point_angle || (navi_yaw < point_angle && navi_yaw <= 0)) {
                Car_speed.linear.x=linear_speed;
                Car_speed.angular.z=angular_speed;
            }
            else {
                Car_speed.linear.x=linear_speed;
                Car_speed.angular.z=-angular_speed;               
            }
        }
        else {

            if(navi_yaw <= point_angle || (navi_yaw > point_angle && navi_yaw > 0)) {
                
                Car_speed.linear.x=linear_speed;
                Car_speed.angular.z=-angular_speed;
            }
            else {
                Car_speed.linear.x=linear_speed;
                Car_speed.angular.z=angular_speed;
            }
        }

    }
    else {
                Car_speed.linear.x=2*linear_speed;
                Car_speed.angular.z=0;
    }
    // std::cout << " speed_0： " << (float)car_speed.speed_0<< "  speed_3: " <<  (float)car_speed.speed_3 << std::endl;

    return  distance;

}


void Car_Navigation::tracking(void) {

    if(!inflection_point.empty()) {
        
        // Eigen::Vector2f pose = path.front();
        // std::cout << "pose: x = " << pose(0) << " y = " << pose(1) <<std::endl;
        float distance = speed_calculate(inflection_point.front());
        if(distance < DISTANCE_THRESHOLD) {
            inflection_point.pop();
            tracking_progress++;
            float progress = float(tracking_progress / path_size);
            // ROS_INFO("tracking_progress: %d tracking progress: %f%%!", tracking_progress, progress);
            ROS_WARN("tracking_progress: %d path_size: %d!", tracking_progress, path_size);
        }
    }
    else {
        tracking_flag = 0;
    }
}




