#include "ros/ros.h"
#include "twin_crawler_controller/NidecMotor.h"

#include "linux/serial.h"
#include "sys/ioctl.h"

#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(5);

    int fd;
    struct termios oldtio, newtio;

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0){
        ROS_ERROR("Serial port error : fd = %d", fd);
        return 1;
    }
    ROS_INFO("Serial port open");

    NidecMotor motor1(fd, 1, 2);

    //NidecMotor::MotorResponse response = motor1.getDeviceID();
    //printf("result : %d\n", response.result);
    //printf("result message : %s\n", response.result_message.c_str());
    
    while(ros::ok()){
        if(motor1.update()){
            motor1.readResponse();
            printf("true\n");
        }
        else{
            printf("false\n");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(fd);
    ROS_INFO("Serial port closed");

    return 0;
}