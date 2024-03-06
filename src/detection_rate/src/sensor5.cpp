#include <ros/ros.h>
#include <sstream>
#include <iostream>
#include <serial/serial.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "detection_rate/Sensor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_port5");
    // 创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<detection_rate::Sensor>("data5", 10);

    detection_rate::Sensor msg;

    // 创建一个serial类
    serial::Serial sp;
    // 创建timeout
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    // 设置要打开的串口名称
    sp.setPort("/dev/ttyUSB4");
    // 设置串口通信的波特率
    sp.setBaudrate(115200);
    // 串口设置timeout
    sp.setTimeout(timeout);

    try
    {
        // 打开串口
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    // 判断串口是否打开成功
    if (sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB4 is opened.");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        // 获取缓冲区内的字节数
        size_t n = sp.available();
        if (n != 0)
        {
            std::string buffer;
            size_t size = 65536;
            std::string eol = "\n";

            n = sp.readline(buffer, size, eol);
            msg.data = buffer;
            pub.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 关闭串口
    sp.close();
    // */
    return 0;
}