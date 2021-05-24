/************************************************************/
/*                                                          */
/* http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom */
/*                                                          */
/************************************************************/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <boost/bind.hpp>
#include <signal.h>

#include "twin_crawler_controller/NidecMotor.h"
#include "twin_crawler_controller/motor_response.h"
#include "twin_crawler_controller/motor_request.h"

const int motor_left_id = 2;
const int motor_right_id = 3;

//const double power_limiter = 1; // 0(停止) ? 1(最高)
//const double joystick_limiter = 1 / sqrt(2);
const double motor_speed_max = 3000;

//const double linear_angular_ratio = 0.8;

const double gear_ratio = 0.01; // ギア比 = 100：1
const double sprocket_pin = 24;
const double sprocket_diameter = 0.295; // [m]
const double belt_pin = 40;
const double belt_length = 1.7; // [m]
const double sprocket_offset = 0.012; // [m]
const double wheel_distance = 0.60; // [m]
const double init_yaw = 0;

// x [deg/s] -> x * gear_ratio / 180.0 * M_PI [rad/s] 
//#define CmdtoOmg(x) (double)(x) * (gear_ratio / 180.0 * M_PI)
//#define OmgtoCmd(x) (int)   (x) / (gear_ratio / 180.0 * M_PI)
//#define OmgtoVel(x) (x) * ((sprocket_diameter+sprocket_offset) / 2.0 * sprocket_pin / belt_pin * belt_length)
//#define VeltoOmg(x) (x) / ((sprocket_diameter+sprocket_offset) / 2.0 * sprocket_pin / belt_pin * belt_length)

// x [rpm]   -> x * gear_ratio / 60.0  * 2.0 * M_PI [rad/s] 
#define CmdtoOmg(x) (double)(x) * (gear_ratio / 60.0 * 2.0 * M_PI)
#define OmgtoCmd(x) (int)  ((x) / (gear_ratio / 60.0 * 2.0 * M_PI))
#define OmgtoVel(x) (x) * ((sprocket_diameter+sprocket_offset) / 2.0) 
#define VeltoOmg(x) (x) / ((sprocket_diameter+sprocket_offset) / 2.0) 
#define CmdtoRad(x) (double)(x) * (gear_ratio * M_PI / 180.0)
#define RadtoCmd(x) (int)  ((x) / (gear_ratio * M_PI / 180.0))

bool error_handler = false;
void mySigintHandler(int sig){
    error_handler = true;
}

class MotorStatus{
    public:
    bool update = false;
    double position = 0;
    double velocity = 0;
};

void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr &msg, ros::ServiceClient *motor_request_client){
    //ROS_INFO("cmd_vel : Linear [%-7.4f, %-7.4f, %-7.4f]", msg->linear.x, msg->linear.y, msg->linear.z);
    //ROS_INFO("cmd_vel : Angular[%-7.4f, %-7.4f, %-7.4f]", msg->angular.x, msg->angular.y, msg->angular.z);

    //int left_speed = msg->linear.x * motor_speed_max * joystick_limiter * power_limiter;
    //int right_speed = msg->linear.x * motor_speed_max * joystick_limiter * power_limiter * -1;
    //left_speed += msg->angular.z * motor_speed_max * joystick_limiter * power_limiter;
    //right_speed -= msg->angular.z * motor_speed_max * joystick_limiter * power_limiter * -1;

    int left_speed  =  OmgtoCmd(VeltoOmg(msg->linear.x - wheel_distance * msg->angular.z / 2.0));
    int right_speed = -OmgtoCmd(VeltoOmg(msg->linear.x + wheel_distance * msg->angular.z / 2.0));

    if (left_speed  < -motor_speed_max) left_speed  = -motor_speed_max;
    if (left_speed  >  motor_speed_max) left_speed  =  motor_speed_max;
    if (right_speed < -motor_speed_max) right_speed = -motor_speed_max;
    if (right_speed >  motor_speed_max) right_speed =  motor_speed_max;

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
    //ROS_INFO("motor_response.id_motor : %d", msg->id_motor);
    //ROS_INFO("motor_response.command  : %s", msg->command_str.c_str());
    //ROS_INFO("motor_response.data     : %d", msg->data);
    //ROS_INFO("motor_response.message  : %s", msg->message.c_str());
    //printf("\n");

    if(msg->command == NidecMotor::Command::readPosition_){
        switch(msg->id_motor){
            case motor_left_id:
            motor_left_status->update = true;
            motor_left_status->position  =  CmdtoRad((double)msg->data);
            //printf("L %lf\n",motor_left_status->position*180.0/M_PI);
            break;

            case motor_right_id:
            motor_right_status->update = true;
            motor_right_status->position = -CmdtoRad((double)msg->data);
            //printf("R %lf\n",motor_left_status->position*180.0/M_PI);
            break;

            default:
            ROS_WARN("Unknown Motor ID");
            break;
        }
    }

    if(msg->command == NidecMotor::Command::readSpeed_){
        switch(msg->id_motor){
            case motor_left_id:
            motor_left_status->update = true;
            motor_left_status->velocity  =  OmgtoVel(CmdtoOmg((double)msg->data));
            //printf("L %lf\n",motor_left_status->velocity);
            break;

            case motor_right_id:
            motor_right_status->update = true;
            motor_right_status->velocity = -OmgtoVel(CmdtoOmg((double)msg->data));
            //printf("R %lf\n",motor_left_status->velocity);
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

    MotorStatus motor_left_status;
    MotorStatus motor_right_status;
    ros::Subscriber motor_response_subscriber = nh.subscribe<twin_crawler_controller::motor_response>("motor_response", 100, boost::bind(&callback_motor_response, _1, &motor_left_status, &motor_right_status));
    ros::ServiceClient motor_request_client = nh.serviceClient<twin_crawler_controller::motor_request>("motor_request");
    ros::Subscriber cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&callback_cmd_vel, _1, &motor_request_client));
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
    tf2_ros::TransformBroadcaster odom_broadcaster;

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
        //ros::spinOnce();

        ROS_INFO("Reset Error");
        req.request.command = NidecMotor::Command::resetError_;
        req.request.id_motor = motor_left_id;
        motor_request_client.call(req);
        req.request.id_motor = motor_right_id;
        motor_request_client.call(req);
        sleep(1);
        //ros::spinOnce();

        ROS_INFO("Motor Encoder Offset... (please wait 15 sec)");
        req.request.command = NidecMotor::Command::offsetEncoder_;
        req.request.id_motor = motor_left_id;
        motor_request_client.call(req);
        req.request.id_motor = motor_right_id;
        motor_request_client.call(req);
        sleep(15);
        //ros::spinOnce();
    }

    ROS_INFO("Write Motor Position : 0");
    req.request.command = NidecMotor::Command::writePosition_;
    req.request.data = 0;
    req.request.id_motor = motor_left_id;
    motor_request_client.call(req);
    req.request.id_motor = motor_right_id;
    motor_request_client.call(req);
    sleep(1);
    //ros::spinOnce();

    ROS_INFO("Motor Mode Change : Speed");
    req.request.command = NidecMotor::Command::writeControlMode_;
    req.request.data = NidecMotor::ControlMode::Speed;
    req.request.id_motor = motor_left_id;
    motor_request_client.call(req);
    req.request.id_motor = motor_right_id;
    motor_request_client.call(req);
    sleep(1);
    //ros::spinOnce();

    ROS_INFO("LOOP START !!");

    double x = 0.0;
    double y = 0.0;
    double th = init_yaw;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(100.0);

    while(ros::ok()){
        //ROS_INFO("loop");

        ros::spinOnce(); 

        /*
        req.request.command = NidecMotor::Command::readPosition_;
        req.request.id_motor = motor_left_id;
        motor_request_client.call(req);
        req.request.id_motor = motor_right_id;
        motor_request_client.call(req);
        */

        req.request.command = NidecMotor::Command::readSpeed_;
        req.request.id_motor = motor_left_id;
        motor_request_client.call(req);
        req.request.id_motor = motor_right_id;
        motor_request_client.call(req);

        if(motor_left_status.update && motor_right_status.update){
            current_time = ros::Time::now();

            double vx = (motor_left_status.velocity + motor_right_status.velocity) / 2.0;
            vth = (-motor_left_status.velocity + motor_right_status.velocity) / wheel_distance;

            double dt = (current_time - last_time).toSec();
            double delta_x = vx * cos(th) * dt;
            double delta_y = vx * sin(th) * dt;
            double delta_th = vth * dt;

            x += delta_x;
            y += delta_y;
            th += delta_th;

            tf2::Quaternion quat_tf;
            quat_tf.setRPY(0.0, 0.0, th);
            geometry_msgs::Quaternion odom_quat;
            tf2::convert(quat_tf, odom_quat);

            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            odom_broadcaster.sendTransform(odom_trans);

            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";

            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            odom.pose.covariance[0] = 1e-2;
            odom.pose.covariance[7] = 1e-2;
            odom.pose.covariance[14] = 1e5;
            odom.pose.covariance[21] = 1e5;
            odom.pose.covariance[28] = 1e5;
            odom.pose.covariance[35] = 1e-2;

            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vth;
            odom.twist.covariance[0] = 1e-2;
            odom.twist.covariance[7] = 1e-2;
            odom.twist.covariance[14] = 1e5;
            odom.twist.covariance[21] = 1e5;
            odom.twist.covariance[28] = 1e5;
            odom.twist.covariance[35] = 1e-2;

            odom_pub.publish(odom);

            last_time = current_time;

            motor_left_status.update = false;
            motor_right_status.update = false;

            //printf("%lf\t%lf\t%lf\n",x, y, th*180.0/M_PI);
            //printf("time %lf\n",dt);
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

        r.sleep();
    }

    printf("Finish Loop\n");

    return 0;
}
