#include "Serial_Send.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Bool.h"
#define AUTO_MODE 1
#define HAND_MODE 0

ros::Subscriber _mode_sub, _speed_sub;
Serial_Send_Msg * _send_msg = new Serial_Send_Msg();

int Ctl_Mode = HAND_MODE;
geometry_msgs::Twist Auto_Speed;
geometry_msgs::Twist Hand_speed;
std_msgs::Bool  End_Ros;
int scanKeyboard(void);
void Speed_Set(int Key_Value);

void recClickedCallBack(const geometry_msgs::PointStamped::ConstPtr& msg) {

    Ctl_Mode = HAND_MODE;
}

void recSpeedCallBack(const geometry_msgs::Twist& msg) {

    Auto_Speed.linear=msg.linear;
    Auto_Speed.angular=msg.angular;     
}

int main(int argc, char **argv) {
    End_Ros.data=false;//

    ros::init(argc, argv, "car_ctl");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    ros::Publisher end_ros_pub = nh.advertise<std_msgs::Bool>("/end_ROS", 50);

    _mode_sub = nh.subscribe("/clicked_point", 1, recClickedCallBack); 
    //  _speed_sub = nh.subscribe("/car_speed", 4, recSpeedCallBack);
    _speed_sub = nh.subscribe("/my_cmd_vel", 1, recSpeedCallBack);
    

    int Key_Value = 0;
    ros::Rate loop_rate(50);
     geometry_msgs::Twist  speed;
    while(ros::ok())
    {   
        if(Ctl_Mode == HAND_MODE) {
            Speed_Set(scanKeyboard()); 
            vel_pub.publish(Hand_speed);
            // ROS_INFO("Mode: %d,HAND_MODE", Ctl_Mode);
        }
        else {

            Speed_Set(scanKeyboard()); 
            vel_pub.publish(Auto_Speed);
             ROS_INFO("Mode:Auto_Speed.linear,Auto_Speed.angular.z,%f,%f", Auto_Speed.linear.x,Auto_Speed.angular.z);  
            
            // ROS_INFO("Mode: %d,AUTO_MODE", Ctl_Mode);  
        }   
        vel_pub.publish(End_Ros);//关闭ROS 
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return -1;
}


void  Speed_Set(int Key_Value) {

    float set_speed_linear = 0.15;
    float set_speed_angular = 0.27;
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    linear.y = 0.0;
    linear.z = 0.0;
    angular.x = 0.0;
    angular.y = 0.0;

    switch(Key_Value) {

        case KEY_W:
            linear.x = set_speed_linear;
            angular.z =0;      
            break;
        case KEY_A:
            linear.x = 0;
            angular.z =set_speed_angular;             
            break;
        case KEY_S:
            linear.x = -set_speed_linear;
            angular.z =0;              
            break;
        case KEY_D:
            linear.x = 0;
            angular.z =-set_speed_angular;            
            break;
        case KEY_Q:
            linear.x =set_speed_linear+0.03;
            angular.z =0;             
            break;
        case KEY_Z:
            linear.x =set_speed_linear-0.03;
            angular.z =0;             
            break;
        case KEY_E:
            linear.x =0;
            angular.z =set_speed_angular+0.03;            
            break;     
        case KEY_C:
            linear.x =0;
            angular.z =set_speed_angular-0.03;            
            break;  
        case KEY_X:
            linear.x = 0;
            angular.z =0;        
            break;       
        case KEY_J:
            Ctl_Mode = AUTO_MODE;
            ROS_INFO("Mode: %d,AUTO_MODE", Ctl_Mode);
            break; 
        case KEY_H:
            Ctl_Mode = HAND_MODE;
            ROS_INFO("Mode: %d,HAND_MODE", Ctl_Mode);
            break; 
        case KEY_Esc:
            End_Ros.data=true;
            ROS_INFO("-------------关闭ROS!!!-----------------------");
            break;  
           
            
    }

    Hand_speed.linear=linear;
    Hand_speed.angular=angular;
}

int scanKeyboard(void)
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    
    in = getchar();
    
    tcsetattr(0,TCSANOW,&stored_settings);
    return in;
}
