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

#define fd1_address "/dev/ttyUSB-MotorL"

bool motorL_lock = false;

//bool requestCallback(twin_crawler_controller::motor_request::Request &req, twin_crawler_controller::motor_request::Response &res){
bool requestCallback(twin_crawler_controller::motor_request::Request &req, twin_crawler_controller::motor_request::Response &res, NidecMotor motorL){
    if(req.id_motor == 2){
        if(motorL_lock){
            res.result = 0;
            res.result_message = "motorL is Locked...";
        }
        else{
            res.result = 1;
            res.result_message = "Request Accept";
            switch(req.command){
                case NidecMotor::Command::run_:
                motorL.run();
                break;

                case NidecMotor::Command::stop_:
                motorL.stop();
                break;

                case NidecMotor::Command::emmergencyStop_:
                motorL.emmergencyStop();
                break;

                case NidecMotor::Command::breakCommand_:
                motorL.breakCommand();
                break;

                case NidecMotor::Command::servoOn_:
                motorL.servoOn();
                break;

                case NidecMotor::Command::servoOff_:
                motorL.servoOff();
                break;

                case NidecMotor::Command::getErrorInfo_:
                motorL.getErrorInfo();
                break;

                case NidecMotor::Command::resetError_:
                motorL.resetError();
                break;

                case NidecMotor::Command::checkConnection_:
                motorL.checkConnection();
                break;

                case NidecMotor::Command::readDeviceID_:
                motorL.readDeviceID();
                break;

                case NidecMotor::Command::readControlMode_:
                motorL.readControlMode();
                break;

                case NidecMotor::Command::writeControlMode_:
                motorL.writeControlMode((NidecMotor::ControlMode)req.data);
                break;

                case NidecMotor::Command::offsetEncoder_:
                motorL.offsetEncoder();
                break;

                case NidecMotor::Command::spinMotor_:
                motorL.spinMotor((int)req.data);
                break;

                default:
                res.result = 0;
                res.result_message = "Not defined yet";
                break;
            }
            //motorL_lock = true;
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
    
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle nh;
    
    /*
    int fd;
    struct termios oldtio, newtio;

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0){
        ROS_ERROR("Serial port error : fd = %d", fd);
        return 1;
    }
    */
    int fd1 = openSerial(fd1_address);
    if(fd1 < 0){
        ROS_ERROR("Serial port error : fd = %d", fd1);
        return 1;
    }
    ROS_INFO("Serial port open : fd = %d", fd1);

    NidecMotor motorL(fd1, 1, 2);

    //ros::ServiceServer motor_request_subscriber = nh.advertiseService("motor_request", requestCallback);
    ros::ServiceServer motor_request_subscriber = nh.advertiseService<twin_crawler_controller::motor_request::Request, twin_crawler_controller::motor_request::Response>("motor_request", boost::bind(&requestCallback, _1, _2, motorL));
    ros::Publisher motor_response_publisher = nh.advertise<twin_crawler_controller::motor_response>("motor_response", 100);

    ros::Rate loop_rate(5);
    ROS_INFO("Start");

    //NidecMotor::MotorResponse response = motorL.getDeviceID();
    //printf("result : %d\n", response.result);
    //printf("result message : %s\n", response.result_message.c_str());
    while(ros::ok()){
        //ROS_INFO("roop");
        //printf("roop\n");
        if(motorL.update()){
            printf("true\n");
            NidecMotor::MotorResponse response = motorL.readResponse();
            twin_crawler_controller::motor_response topic_response;
            topic_response.id_motor = 2;
            topic_response.command = response.ack & 0x3f;
            topic_response.result = response.result;
            topic_response.result_message = response.result_message;
            topic_response.ack = response.ack;
            topic_response.ack_message = response.ack_message;
            topic_response.data = response.data;
            motor_response_publisher.publish(topic_response);
            motorL_lock = false;
        }
        else{
            printf("false\n");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(fd1);
    //ROS_INFO("Serial port closed");
    printf("Serial port closed\n");

    return 0;
}
