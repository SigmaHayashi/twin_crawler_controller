#include "ros/ros.h"
#include "twin_crawler_controller/NidecMotor.h"

#include "linux/serial.h"
#include "sys/ioctl.h"

#include <stdio.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "twin_crawler_controller/motor_response.h"
#include "twin_crawler_controller/motor_request.h"

#include <boost/bind.hpp>

bool motor1_lock = false;

//bool requestCallback(twin_crawler_controller::motor_request::Request &req, twin_crawler_controller::motor_request::Response &res){
bool requestCallback(twin_crawler_controller::motor_request::Request &req, twin_crawler_controller::motor_request::Response &res, NidecMotor motor1){
    if(req.id_motor == 1){
        if(motor1_lock){
            res.result = 0;
            res.result_message = "Motor1 is Locked...";
        }
        else{
            res.result = 1;
            res.result_message = "Request Accept";
            switch(req.command){
                case NidecMotor::Command::run_:
                motor1.run();
                break;

                case NidecMotor::Command::stop_:
                motor1.stop();
                break;

                case NidecMotor::Command::emmergencyStop_:
                motor1.emmergencyStop();
                break;

                case NidecMotor::Command::breakCommand_:
                motor1.breakCommand();
                break;

                case NidecMotor::Command::servoOn_:
                motor1.servoOn();
                break;

                case NidecMotor::Command::servoOff_:
                motor1.servoOff();
                break;

                case NidecMotor::Command::getErrorInfo_:
                motor1.getErrorInfo();
                break;

                case NidecMotor::Command::resetError_:
                motor1.resetError();
                break;

                case NidecMotor::Command::checkConnection_:
                motor1.checkConnection();
                break;

                case NidecMotor::Command::readDeviceID_:
                motor1.readDeviceID();
                break;

                case NidecMotor::Command::readControlMode_:
                motor1.readControlMode();
                break;

                case NidecMotor::Command::writeControlMode_:
                motor1.writeControlMode((NidecMotor::ControlMode)req.data);
                break;

                case NidecMotor::Command::offsetEncoder_:
                motor1.offsetEncoder();
                break;

                case NidecMotor::Command::spinMotor_:
                motor1.spinMotor((int)req.data);
                break;

                default:
                res.result = 0;
                res.result_message = "Not defined yet";
                break;
            }
            motor1_lock = true;
        }
    }

    return true;
}

int openSerial(const char *device_name){
    int fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd, F_SETFL, 0);

    struct termios conf_tio;
    tcgetattr(fd, &conf_tio);

    speed_t BAUDRATE = B115200;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);

    conf_tio.c_cflag &= ~(ECHO | ICANON);

    conf_tio.c_cc[VMIN] = 0;
    conf_tio.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &conf_tio);
    
    return fd;
}

int main(int argc, char **argv){

    /*
    int fd;
    struct termios oldtio, newtio;

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0){
        ROS_ERROR("Serial port error : fd = %d", fd);
        return 1;
    }
    */
    int fd1 = openSerial("/dev/ttyUSB0");
    if(fd1 < 0){
        ROS_ERROR("Serial port error : fd = %d", fd1);
        return 1;
    }
    ROS_INFO("Serial port open");

    NidecMotor motor1(fd1, 1, 2);

    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    //ros::ServiceServer motor_request_subscriber = nh.advertiseService("motor_request", requestCallback);
    ros::ServiceServer motor_request_subscriber = nh.advertiseService<twin_crawler_controller::motor_request::Request, twin_crawler_controller::motor_request::Response>("motor_request", boost::bind(&requestCallback, _1, _2, motor1));
    ros::Publisher motor_response_publisher = nh.advertise<twin_crawler_controller::motor_response>("motor_response", 100);

    ros::Rate loop_rate(5);

    //NidecMotor::MotorResponse response = motor1.getDeviceID();
    //printf("result : %d\n", response.result);
    //printf("result message : %s\n", response.result_message.c_str());
    
    while(ros::ok()){
        if(motor1.update()){
            printf("true\n");
            NidecMotor::MotorResponse response = motor1.readResponse();
            twin_crawler_controller::motor_response topic_response;
            topic_response.id_motor = 1;
            topic_response.result = response.result;
            topic_response.result_message = response.result_message;
            topic_response.ack = response.ack;
            topic_response.ack_message = response.ack_message;
            topic_response.data = response.data;
            motor_response_publisher.publish(topic_response);
            motor1_lock = false;
        }
        else{
            printf("false\n");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(fd1);
    ROS_INFO("Serial port closed");

    return 0;
}