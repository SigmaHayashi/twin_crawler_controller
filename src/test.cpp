#include "ros/ros.h"
#include "twin_crawler_controller/NidecMotor.h"

#include "linux/serial.h"
#include "sys/ioctl.h"

#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#include "twin_crawler_controller/motor_response.h"
#include "twin_crawler_controller/motor_request.h"

bool requestCallback(twin_crawler_controller::motor_request::Request &req, twin_crawler_controller::motor_request::Response &res){
    res.result = 0;
    res.result_message = "Not defined yet";
    return true;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::ServiceServer motor_request_subscriber = nh.advertiseService("motor_request", requestCallback);
    ros::Publisher motor_response_publisher = nh.advertise<twin_crawler_controller::motor_response>("motor_response", 100);

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
            printf("true\n");
            NidecMotor::MotorResponse response = motor1.readResponse();
            twin_crawler_controller::motor_response topic_response;
            topic_response.result = response.result;
            topic_response.result_message = response.result_message;
            topic_response.ack = response.ack;
            topic_response.ack_message = response.ack_message;
            topic_response.data = response.data;
            motor_response_publisher.publish(topic_response);
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