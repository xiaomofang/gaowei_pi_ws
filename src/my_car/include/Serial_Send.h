#ifndef SERIAL_SEND
#define SERIAL_SEND

#include <serial/serial.h>
#include <ros/ros.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#define PORT "/dev/ttyUSB1"
#define HEAD_DATA   0X55
#define CHECK_DATA1 0X00
#define CHECK_DATA2 0XFF
#define SPEED_ADD   0X64

#define KEY_W 119
#define KEY_A 97
#define KEY_S 115
#define KEY_D 100
#define KEY_Q 113
#define KEY_Z 122
#define KEY_E 101
#define KEY_C 99
#define KEY_X 120
#define KEY_H 104
#define KEY_J 106
#define KEY_Esc 27

class Serial_Send_Msg
{
private:
    /* data */
    serial::Serial sp;

public:
    
    Serial_Send_Msg(/* args */);
    bool Serial_Open();
    void Serial_Close();
    void Send_Speed(int8_t *speed);    

};




#endif


