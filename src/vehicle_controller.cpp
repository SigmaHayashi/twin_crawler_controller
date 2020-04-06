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

const int motor_left_id = 2;
const int motor_right_id = 3;

const float power_limiter = 1; // 0(停止) 〜 1(最高)
const float joystick_limiter = 1 / sqrt(2);
const float motor_speed_max = 3000;

const float linear_angular_ratio = 0.8;

const float gear_ratio = 0.01; // ギア比 = 100：1
const float sprocket_pin = 24;
const float sprocket_diameter = 0.295; // [m]
const float belt_pin = 40;
const float belt_length = 1.7; // [m]

bool error_handler = false;
void mySigintHandler(int sig){
    error_handler = true;
}

class MotorStatus{
    public:
    bool position_update = false;
    float position = 0;
    float position_old = 0;
};

void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr &msg, ros::ServiceClient *motor_request_client){
    //ROS_INFO("cmd_vel : Linear [%-7.4f, %-7.4f, %-7.4f]", msg->linear.x, msg->linear.y, msg->linear.z);
    //ROS_INFO("cmd_vel : Angular[%-7.4f, %-7.4f, %-7.4f]", msg->angular.x, msg->angular.y, msg->angular.z);

    int left_speed = msg->linear.x * motor_speed_max * joystick_limiter * power_limiter;
    int right_speed = msg->linear.x * motor_speed_max * joystick_limiter * power_limiter * -1;

    left_speed += msg->angular.z * motor_speed_max * joystick_limiter * power_limiter;
    right_speed -= msg->angular.z * motor_speed_max * joystick_limiter * power_limiter * -1;

    //ROS_INFO("Motor Roll Speed [rpm] : %d, %d", left_speed, right_speed);

    twin_crawler_controller::motor_request req;
    req.request.id_motor = motor_left_id;
    req.request.command = NidecMotor::Command::rollBySpeed_;
    req.request.data = left_speed;
    motor_request_client->call(req);

    req.request.id_motor = motor_right_id;
    req.request.command = NidecMotor::Command::rollBySpeed_;
    req.request.data = right_speed;
    motor_request_client->call(req);
}

void callback_motor_response(const twin_crawler_controller::motor_response::ConstPtr &msg, MotorStatus *motor_left_status, MotorStatus *motor_right_status){
    ROS_INFO("motor_response.id_motor : %d", msg->id_motor);
    ROS_INFO("motor_response.command  : %s", msg->command_str.c_str());
    ROS_INFO("motor_response.data     : %d", msg->data);
    ROS_INFO("motor_response.message  : %s", msg->message.c_str());
    printf("\n");

    if(msg->command == NidecMotor::Command::readPosition_){
        switch(msg->id_motor){
            case motor_left_id:
            motor_left_status->position_update = true;
            //motor_left_status->position = msg->data; //要変更
            motor_left_status->position = msg->data * gear_ratio / 360 * M_PI * sprocket_diameter * sprocket_pin / belt_pin * belt_length;
            break;

            case motor_right_id:
            motor_right_status->position_update = true;
            //motor_right_status->position = msg->data; //要変更
            motor_right_status->position = msg->data * gear_ratio / 360 * M_PI * sprocket_diameter * sprocket_pin / belt_pin * belt_length * -1;
            break;

            default:
            ROS_WARN("Unknown Motor ID");
            break;
        }
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "vehicle_controller");
    ros::NodeHandle nh;

    ROS_INFO("Node Start");

    ros::Rate loop_rate(10);

    MotorStatus motor_left_status;
    MotorStatus motor_right_status;
    ros::Subscriber motor_response_subscriber = nh.subscribe<twin_crawler_controller::motor_response>("motor_response", 100, boost::bind(&callback_motor_response, _1, &motor_left_status, &motor_right_status));
    ros::ServiceClient motor_request_client = nh.serviceClient<twin_crawler_controller::motor_request>("motor_request");

    ros::Subscriber cmd_vdl_subscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 100, boost::bind(&callback_cmd_vel, _1, &motor_request_client));

    ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>("vehicle_odometry", 100);

    bool serial_port_open;
    while(!serial_port_open && ros::ok()){
        nh.getParam("serial_port_open", serial_port_open);
        if(!serial_port_open){
            ROS_ERROR("Serial port is not opened...");
            sleep(1);
        }
    }
    if(!ros::ok()){
        return 1;
    }

    twin_crawler_controller::motor_request req;

    //ROS_ERROR("argc    : %d", argc);
    //ROS_ERROR("argv[1] : %s", argv[1]);

    if(argc < 2 || argv[1][0] != '0'){
        ROS_INFO("Motor Mode Change : Release");
        req.request.command = NidecMotor::Command::writeControlMode_;
        req.request.data = NidecMotor::ControlMode::Release;
        req.request.id_motor = motor_left_id;
        motor_request_client.call(req);
        req.request.id_motor = motor_right_id;
        motor_request_client.call(req);
        sleep(1);
        ros::spinOnce();

        ROS_INFO("Reset Error");
        req.request.command = NidecMotor::Command::resetError_;
        req.request.id_motor = motor_left_id;
        motor_request_client.call(req);
        req.request.id_motor = motor_right_id;
        motor_request_client.call(req);
        sleep(1);
        ros::spinOnce();

        ROS_INFO("Motor Encoder Offset... (please wait 15 sec)");
        req.request.command = NidecMotor::Command::offsetEncoder_;
        req.request.id_motor = motor_left_id;
        motor_request_client.call(req);
        req.request.id_motor = motor_right_id;
        motor_request_client.call(req);
        sleep(15);
        ros::spinOnce();
    }

    ROS_INFO("Write Motor Position : 0");
    req.request.command = NidecMotor::Command::writePosition_;
    req.request.data = 0;
    req.request.id_motor = motor_left_id;
    motor_request_client.call(req);
    req.request.id_motor = motor_right_id;
    motor_request_client.call(req);
    sleep(1);
    ros::spinOnce();

    ROS_INFO("Motor Mode Change : Speed");
    req.request.command = NidecMotor::Command::writeControlMode_;
    req.request.data = NidecMotor::ControlMode::Speed;
    req.request.id_motor = motor_left_id;
    motor_request_client.call(req);
    req.request.id_motor = motor_right_id;
    motor_request_client.call(req);
    sleep(1);
    ros::spinOnce();

    ROS_INFO("LOOP START !!");

    nav_msgs::Odometry odometry;

    while(ros::ok()){
        //ROS_INFO("loop");

        /*
        req.request.command = NidecMotor::Command::readSpeed_;
        req.request.id_motor = motor_left_id;
        motor_request_client.call(req);
        req.request.id_motor = motor_right_id;
        motor_request_client.call(req);
        */
        
        req.request.command = NidecMotor::Command::readPosition_;
        req.request.id_motor = motor_left_id;
        motor_request_client.call(req);
        req.request.id_motor = motor_right_id;
        motor_request_client.call(req);
        
        if(motor_left_status.position_update && motor_right_status.position_update){
            //要変更
            /*
            geometry_msgs::Twist odd_data;
            odd_data.linear.x = motor_left_status.position;
            odd_data.linear.y = motor_right_status.position;
            odd_publisher.publish(odd_data);
            */
            
            //odometry.twist.twist.linear.x = (motor_left_status.position - odometry.pose.pose.position.x) / (ros::Time::now() - odometry.header.stamp).toSec();
            //odometry.twist.twist.linear.y = (motor_right_status.position - odometry.pose.pose.position.y) / (ros::Time::now() - odometry.header.stamp).toSec();
            float time_difference = (ros::Time::now() - odometry.header.stamp).toSec();
            float crawler_left_speed = (motor_left_status.position - motor_left_status.position_old) / time_difference;
            float crawler_right_speed = (motor_right_status.position - motor_right_status.position_old) / time_difference;
            //if(crawler_left_speed * crawler_right_speed > 0){
            //}
            if(abs(crawler_left_speed) < abs(crawler_right_speed)){
                odometry.twist.twist.linear.x = crawler_left_speed;
            }
            else{
                odometry.twist.twist.linear.x = crawler_right_speed;
            }

            odometry.twist.twist.angular.z = crawler_right_speed - crawler_left_speed;

            //odometry.pose.pose.position.x = motor_left_status.position;
            //odometry.pose.pose.position.y = motor_right_status.position;
            odometry.pose.pose.position.x = odometry.pose.pose.position.x + odometry.twist.twist.linear.x * cos(odometry.twist.twist.angular.z) * time_difference;
            odometry.pose.pose.position.y = odometry.pose.pose.position.y + odometry.twist.twist.linear.x * sin(odometry.twist.twist.angular.z) * time_difference;

            odometry.pose.pose.orientation.z = odometry.pose.pose.orientation.z + odometry.twist.twist.angular.z * time_difference;

            odometry.header.stamp = ros::Time::now();
            odometry_publisher.publish(odometry);

            motor_left_status.position_update = false;
            motor_right_status.position_update = false;
            motor_left_status.position_old = motor_left_status.position;
            motor_right_status.position_old = motor_right_status.position;
        }

        signal(SIGINT, mySigintHandler);
        if(error_handler){
            ROS_INFO("Ctrl + C received");

            sleep(1);
            ROS_INFO("Motor Stop");
            req.request.command = NidecMotor::Command::stop_;
            req.request.id_motor = motor_left_id;
            motor_request_client.call(req);
            req.request.id_motor = motor_right_id;
            motor_request_client.call(req);
            
            ros::shutdown();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    printf("Finish Loop\n");

    return 0;
}
