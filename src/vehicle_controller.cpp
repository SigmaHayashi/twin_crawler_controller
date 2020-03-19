#include "ros/ros.h"
#include "twin_crawler_controller/NidecMotor.h"

#include <boost/bind.hpp>

#include <signal.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"

#include "twin_crawler_controller/motor_response.h"
#include "twin_crawler_controller/motor_request.h"

bool error_handler = false;
void mySigintHandler(int sig){
    error_handler = true;
    /*
    ROS_INFO("Motor Stop");
    twin_crawler_controller::motor_request req;
    req.request.command = NidecMotor::Command::stop_;
    req.request.id_motor = 2;
    motor_request_client->call(req);
    req.request.id_motor = 3;
    motor_request_client->call(req);

    ros::shutdown();
    */
}

void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr &msg, ros::ServiceClient *motor_request_client){
    ROS_INFO("cmd_vel : Linear [%-7.4f, %-7.4f, %-7.4f]", msg->linear.x, msg->linear.y, msg->linear.z);
    ROS_INFO("cmd_vel : Angular[%-7.4f, %-7.4f, %-7.4f]", msg->angular.x, msg->angular.y, msg->angular.z);

    int left_speed = msg->linear.x * 3000 * 0.5;
    int right_speed = msg->linear.x * -3000 * 0.5;

    if(msg->angular.z > 0){
        left_speed += msg->angular.z * 3000 * 0.5;
        right_speed -= msg->angular.z * -3000 * 0.5;
    }
    else if(msg->angular.z < 0){
        left_speed -= msg->angular.z * -1 * 3000 * 0.5;
        right_speed += msg->angular.z * -1 * -3000 * 0.5;
    }

    ROS_INFO("Motor Roll Speed [rpm] : %d, %d", left_speed, right_speed);

    twin_crawler_controller::motor_request req;
    req.request.id_motor = 2;
    req.request.command = NidecMotor::Command::rollBySpeed_;
    req.request.data = left_speed;
    motor_request_client->call(req);

    req.request.id_motor = 3;
    req.request.command = NidecMotor::Command::rollBySpeed_;
    req.request.data = right_speed;
    motor_request_client->call(req);
}

void callback_motor_response(const twin_crawler_controller::motor_response &msg){
    ROS_INFO("motor_response.result_message : %s", msg.result_message.c_str());
    ROS_INFO("motor_response.data           : %ld", msg.data);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "vehicle_controller");
    ros::NodeHandle nh;

    ROS_INFO("Start");

    ros::Rate loop_rate(5);

    ros::Subscriber motor_response_subscriber = nh.subscribe("motor_response", 100, callback_motor_response);
    ros::ServiceClient motor_request_client = nh.serviceClient<twin_crawler_controller::motor_request>("motor_request");

    ros::Subscriber cmd_vdl_subscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, boost::bind(&callback_cmd_vel, _1, &motor_request_client));

    twin_crawler_controller::motor_request req;
    req.request.command = NidecMotor::Command::writeControlMode_;
    req.request.data = NidecMotor::ControlMode::Release;
    req.request.id_motor = 2;
    motor_request_client.call(req);
    req.request.id_motor = 3;
    motor_request_client.call(req);
    sleep(1);

    req.request.command = NidecMotor::Command::offsetEncoder_;
    req.request.id_motor = 2;
    motor_request_client.call(req);
    req.request.id_motor = 3;
    motor_request_client.call(req);

    ROS_INFO("Motor Encoder Offset...");
    //ros::Rate wait_rate(0.1);
    //wait_rate.sleep();
    sleep(10);
    ROS_INFO("Finish");

    req.request.command = NidecMotor::Command::writeControlMode_;
    req.request.data = NidecMotor::ControlMode::Speed;
    req.request.id_motor = 2;
    motor_request_client.call(req);
    req.request.id_motor = 3;
    motor_request_client.call(req);
    ROS_INFO("Motor Mode Change : Speed");
    sleep(1);

    while(ros::ok()){
        //ROS_INFO("loop");

        //twin_crawler_controller::motor_request req;
        req.request.id_motor = 2;
        req.request.command = NidecMotor::Command::readSpeed_;
        if(motor_request_client.call(req)){
            ROS_INFO("motor_request.result_message : %s", req.response.result_message.c_str());
        }
        else{
            ROS_WARN("motor_request was not returned");
        }

        signal(SIGINT, mySigintHandler);
        if(error_handler){
            ROS_INFO("Motor Stop");
            sleep(1);
            req.request.command = NidecMotor::Command::stop_;
            req.request.id_motor = 2;
            motor_request_client.call(req);
            ROS_INFO("Motor L Stop");
            req.request.id_motor = 3;
            motor_request_client.call(req);
            ROS_INFO("Motor R Stop");

            ros::shutdown();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    printf("Finish Loop\n");

    return 0;
}
