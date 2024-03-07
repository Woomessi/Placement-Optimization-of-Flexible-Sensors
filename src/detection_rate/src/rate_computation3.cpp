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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace message_filters;
double total_time = 0;
double successful_time = 0;
double successful_rate = 0;

void doMsg(const detection_rate::Sensor::ConstPtr &msgp1, const detection_rate::Sensor::ConstPtr &msgp2, const detection_rate::Sensor::ConstPtr &msgp3)
{    
    int msgp1_data = atoi(msgp1->data.c_str());
    int msgp2_data = atoi(msgp2->data.c_str());
    int msgp3_data = atoi(msgp3->data.c_str());

    total_time++;
    if (msgp1_data == 1 || msgp2_data == 1 || msgp3_data == 1)
    {
        successful_time++;
    }
    successful_rate = successful_time/total_time;
    ROS_INFO("successful_rate: %f, successful_time: %f, total_time: %f", successful_rate, successful_time, total_time);
}

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "listener3");
    ros::NodeHandle nh;         
    
    message_filters::Subscriber<detection_rate::Sensor> sensor1_sub(nh, "data1", 10);
    message_filters::Subscriber<detection_rate::Sensor> sensor2_sub(nh, "data2", 10);
    message_filters::Subscriber<detection_rate::Sensor> sensor3_sub(nh, "data3", 10);

    TimeSynchronizer<detection_rate::Sensor, detection_rate::Sensor, detection_rate::Sensor> sync(sensor1_sub, sensor2_sub, sensor3_sub, 10);    
    sync.registerCallback(boost::bind(&doMsg, _1, _2, _3));

    ros::spin();

    return 0;
}   