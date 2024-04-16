#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define PI 3.14159265f

// Basic ROS configuration
const char node_name[] = "encoder_to_odometry_publisher"; // node name
const char encoder_topic[] = "/stm32/encoders";           // publisher
const char odom_topic[] = "/odom";                        // subscriber
const char speed_topic[] = "/cmd_vel";                    // subscriber
const char source_link[] = "odom";                        // TF pair
const char target_link[] = "base_link";                   // TF pair
const int ros_update_hz = 5;                              // in Hz

// Robot physical parameters
const double offset_distance_error = 0.0f;                                                         // distance compensation
const double offset_theta_error = (PI / 406.0f) * 2.0f * 1.05f;                                    // theta compensation
const double robot_wheel_separation = 0.428f;                                                      // in meter
const double robot_wheel_radius = 0.0625f;                                                         // in meter
const double encoder_per_rotation_m1 = 1327.0f;                                                    // encoder value per rotation of motor1 [from 65535 - 65305]
const double encoder_per_rotation_m2 = 1327.0f;                                                    // encoder value per rotation of motor2 [from 0 - 28911]
double distance_per_count_m1 = (double)(2.0f * PI * robot_wheel_radius) / encoder_per_rotation_m1; // Distance for an encoder pulse in m
double distance_per_count_m2 = (double)(2.0f * PI * robot_wheel_radius) / encoder_per_rotation_m2; // Distance for an encoder pulse in m

// Variables
geometry_msgs::Twist msg;
ros::Time current_time, last_time; // ROS Time
double linear_x = 0;               // robot x
double linear_y = 0;               // robot y
double angular_z = 0;              // robot angular z

// Instantaneous variables
double dm1 = 0;
double dm2 = 0;
double dc = 0;
double dx = 0;
double dy = 0;
double dtheta = 0;
double dt = 0;

// Odometry data
double x = 0;
double y = 0;
double theta = 0;

// Previous value
double _x_tick = 0;
double _y_tick = 0;
double _theta = 0;

void VelocityCallback(const geometry_msgs::Twist &msg)
{
    linear_x = msg.linear.x;
    linear_y = msg.linear.y;
    angular_z = msg.angular.z;
}

bool first_time = true;

void EncoderCallback(const geometry_msgs::Vector3::ConstPtr &encoder_ticks)
{
    if (first_time)
    {
        first_time = false;
        // Save past data
        _x_tick = encoder_ticks->x;
        _y_tick = encoder_ticks->y;
        _theta = dtheta;
        return;
    }
    // This calculation assume the encoder will send the value within 0 - 65535 //

    // Time intervals
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    // Number of counts in dt
    // encoder_ticks->x + (65535 - _dm1) used to handle overflow problem of encoder register
    dm1 = (encoder_ticks->x - _x_tick);
    if (dm1 > 30000)
        dm1 -= 65535;
    if (dm1 < -30000)
        dm1 += 65535;
    dm1 = dm1 * distance_per_count_m1;

    dm2 = (encoder_ticks->y - _y_tick);
    if (dm2 > 30000)
        dm2 -= 65535;
    if (dm2 < -30000)
        dm2 += 65535;
    dm2 = dm2 * distance_per_count_m2;

    printf("vx: %f | vy: %f | wz: %f\r\n", linear_x, linear_y, angular_z);
    printf("encoder_1: %f | encoder_2: %f\r\n", encoder_ticks->x, encoder_ticks->y);
    printf("dm1: %f | dm2: %f | dtheta: %f\r\n", dm1, dm2, dtheta);

    // Force compensate two motor
    // dm1 = dm1 * offset_distance_error;

    // Calculate center turning curve
    dc = (dm2 + dm1) * 0.5f;

    // Calculate orientation
    printf("dm1-dm2 = %f \r\n", (dm1 - dm2));
    dtheta = (dm2 - dm1) * 10.0f / robot_wheel_radius;
    // dtheta = 0;

    // Force compensate orientation
    dtheta = dtheta * offset_theta_error;

    // Update odometry data
    if (encoder_ticks->x != _x_tick)
        x += dx;
    if (encoder_ticks->y != _y_tick)
        y += dy;
    if (dtheta != _theta)
        theta += dtheta;

    // Calculate separate distance
    // *1.1 is only for speed up the visualization
    dx = dc * cos(theta);
    dy = dc * sin(theta);

    // Handle angular z polarity
    // (angular_z < 0) ? dtheta = abs(dtheta) *-1 : dtheta = abs(dtheta);
    // // Handle linear x polarity
    // if (linear_x < 0)
    // {
    //     dx = abs(dx) * -1;
    //     dy = abs(dy) * -1;
    // }

    printf("dc: %f | dx: %f | dy: %f | dtheta: %f\r\n", dc, dx, dy, dtheta);
    printf("pose x: %f | pose y: %f | pose theta: %f\r\n", x, y, theta);

    // Save past data
    _x_tick = encoder_ticks->x;
    _y_tick = encoder_ticks->y;
    _theta = dtheta;

    printf("\r\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(encoder_topic, 100, EncoderCallback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 50);
    ros::Subscriber vel_sub = nh.subscribe(speed_topic, 100, VelocityCallback);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Rate r(ros_update_hz);
    printf("Encoder To Odom Transformation started!\r\n");

    double p_gain = 1.0;
    double i_gain = 0.0;
    double d_gain = 0.0;
    bool anti_windup = false;

    // Publish loop
    while (nh.ok())
    {
        // Only Yaw should be considered
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        // A TF should be setup between base_link and odom
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = source_link;
        odom_trans.child_frame_id = target_link;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        // Send the transform
        odom_broadcaster.sendTransform(odom_trans);

        // Publish odom message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = source_link;
        // set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        // set the velocity
        odom.child_frame_id = target_link;
        odom.twist.twist.linear.x = dx / dt;
        odom.twist.twist.linear.y = dy / dt;
        odom.twist.twist.angular.z = dtheta / dt;

        // printf("Position x = %f and Position y = %f ",x,y);

        // publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
