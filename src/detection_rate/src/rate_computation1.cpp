#include <ros/ros.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <serial/serial.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include "detection_rate/Sensor.h"

double total_time = 0;
double successful_time = 0;
double successful_rate = 0;

void doMsg(const detection_rate::Sensor::ConstPtr &msgp1)
{    
    int msgp1_data = atoi(msgp1->data.c_str());
    total_time++;
    if (msgp1_data == 1)
    {
        successful_time++;
    }
    successful_rate = successful_time/total_time;
    ROS_INFO("successful_rate: %f, successful_time: %f, total_time: %f", successful_rate, successful_time, total_time);
}

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "listener1");
    ros::NodeHandle nh;         
    ros::Subscriber sub = nh.subscribe<detection_rate::Sensor>("data1",10, doMsg);
    ros::spin();
    return 0;
}   