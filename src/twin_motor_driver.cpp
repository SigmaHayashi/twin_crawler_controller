#include "ros/ros.h"
#include "twin_crawler_controller/NidecMotor.h"

#include <signal.h>

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

#define fd1_address "/dev/ttyUSB-MotorL"
#define fd2_address "/dev/ttyUSB-MotorR"

const bool use_motor_left = true;
const bool use_motor_right = true;

bool error_handler = false;
void mySigintHandler(int sig){
    error_handler = true;
}

bool requestCallback(twin_crawler_controller::motor_request::Request &req, twin_crawler_controller::motor_request::Response &res, NidecMotor *motor_left, NidecMotor *motor_right){
    NidecMotor *motor;
    switch(req.id_motor){
        case 2:
        if(!use_motor_left){
            res.result = 0;
            res.result_message = "Left Motor is NOT Using";
            return true;
        }
        motor = motor_left;
        break;

        case 3:
        if(!use_motor_right){
            res.result = 0;
            res.result_message = "Right Motor is NOT Using";
            return true;
        }
        motor = motor_right;
        break;

        default:
        res.result = 0;
        res.result_message = "Motor ID Error";
        return true;
    }

    res.result = 1;
    res.result_message = "Request Accept";
    switch(req.command){
        case NidecMotor::Command::run_:
        motor->run();
        break;

        case NidecMotor::Command::stop_:
        motor->stop();
        break;

        case NidecMotor::Command::emmergencyStop_:
        motor->emmergencyStop();
        break;

        case NidecMotor::Command::breakCommand_:
        motor->breakCommand();
        break;

        case NidecMotor::Command::servoOn_:
        motor->servoOn();
        break;

        case NidecMotor::Command::servoOff_:
        motor->servoOff();
        break;

        case NidecMotor::Command::getErrorInfo_:
        motor->getErrorInfo();
        break;

        case NidecMotor::Command::resetError_:
        motor->resetError();
        break;

        case NidecMotor::Command::checkConnection_:
        motor->checkConnection();
        break;

        case NidecMotor::Command::readDeviceID_:
        motor->readDeviceID();
        break;

        case NidecMotor::Command::readControlMode_:
        motor->readControlMode();
        break;

        case NidecMotor::Command::writeControlMode_:
        motor->writeControlMode((NidecMotor::ControlMode)req.data);
        break;

        case NidecMotor::Command::offsetEncoder_:
        motor->offsetEncoder();
        break;

        case NidecMotor::Command::rollBySpeed_:
        motor->rollBySpeed((int)req.data);
        break;

        case NidecMotor::Command::readSpeed_:
        motor->readSpeed();
        break;

        default:
        res.result = 0;
        res.result_message = "Not defined yet";
        break;
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

    conf_tio.c_lflag &= ~(ECHO | ICANON);

    conf_tio.c_cc[VMIN] = 0;
    conf_tio.c_cc[VTIME] = 0;

    conf_tio.c_cc[VINTR] = 0;
    conf_tio.c_cc[VQUIT] = 0;
    conf_tio.c_cc[VERASE] = 0;
    conf_tio.c_cc[VKILL] = 0;
    conf_tio.c_cc[VEOF] = 0;
    conf_tio.c_cc[VSWTC] = 0;
    conf_tio.c_cc[VSTART] = 0;
    conf_tio.c_cc[VSTOP] = 0;
    conf_tio.c_cc[VSUSP] = 0;
    conf_tio.c_cc[VEOL] = 0;
    conf_tio.c_cc[VREPRINT] = 0;
    conf_tio.c_cc[VDISCARD] = 0;
    conf_tio.c_cc[VWERASE] = 0;
    conf_tio.c_cc[VLNEXT] = 0;
    conf_tio.c_cc[VEOL2] = 0;

    tcsetattr(fd, TCSANOW, &conf_tio);
    
    return fd;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "twin_motor_driver");
    ros::NodeHandle nh;
    
    bool serial_error = false;

    int fd1 = openSerial(fd1_address);
    if(use_motor_left){
        if(fd1 < 0){
            ROS_ERROR("MotorL Serial port error : fd1 = %d", fd1);
            serial_error = true;
        }
        else{
            ROS_INFO("MotorL Serial port open : fd1 = %d", fd1);
        }
    }
    
    int fd2 = openSerial(fd2_address);
    if(use_motor_right){
        if(fd2 < 0){
            ROS_ERROR("MotorR Serial port error : fd2 = %d", fd2);
            serial_error = true;
        }
        else{
            ROS_INFO("MotorR Serial port open : fd2 = %d", fd2);
        }
    }

    if(serial_error){
        return 1;
    }

    nh.setParam("serial_port_open", true);

    NidecMotor motor_left(fd1, 1, 2);
    NidecMotor motor_right(fd2, 1, 3);

    ros::ServiceServer motor_request_subscriber = nh.advertiseService<twin_crawler_controller::motor_request::Request, twin_crawler_controller::motor_request::Response>("motor_request", boost::bind(&requestCallback, _1, _2, &motor_left, &motor_right));
    ros::Publisher motor_response_publisher = nh.advertise<twin_crawler_controller::motor_response>("motor_response", 100);

    //ros::Rate loop_rate(5);
    ROS_INFO("Start");

    while(ros::ok()){
        if(use_motor_left){
            if(motor_left.update()){
                //printf("motor_left true\n");
                NidecMotor::MotorResponse response = motor_left.readResponse();
                twin_crawler_controller::motor_response topic_response;
                topic_response.id_motor = 2;
                topic_response.result = response.result;
                topic_response.result_message = response.result_message;
                topic_response.command = response.command;
                topic_response.ack = response.ack;
                topic_response.ack_message = response.ack_message;
                topic_response.data = response.data;
                motor_response_publisher.publish(topic_response);
            }
            else{
                //printf("motor_left false\n");
            }
        }

        if(use_motor_right){
            if(motor_right.update()){
                //printf("motor_right true\n");
                NidecMotor::MotorResponse response = motor_right.readResponse();
                twin_crawler_controller::motor_response topic_response;
                topic_response.id_motor = 3;
                topic_response.result = response.result;
                topic_response.result_message = response.result_message;
                topic_response.command = response.command;
                topic_response.ack = response.ack;
                topic_response.ack_message = response.ack_message;
                topic_response.data = response.data;
                motor_response_publisher.publish(topic_response);
            }
            else{
                //printf("motor_right false\n");
            }
        }

        signal(SIGINT, mySigintHandler);
        if(error_handler){
            close(fd1);
            close(fd2);
            nh.setParam("serial_port_open", false);
            ROS_INFO("Serial port closed");

            ros::shutdown();
        }

        ros::spinOnce();
        //loop_rate.sleep();
    }

    /*
    close(fd1);
    close(fd2);
    //ROS_INFO("Serial port closed");
    printf("Serial port closed\n");
    */

    return 0;
}
