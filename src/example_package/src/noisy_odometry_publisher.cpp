#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <random>
#include <cmath>

double wheel_radius = 0.033; // TurtleBot3 Burger wheel radius [m]
double wheel_separation = 0.160; // TurtleBot3 Burger wheel separation [m]

double last_left = 0.0, last_right = 0.0;
bool first_joint = true;

double x = 0.0, y = 0.0, th = 0.0;
double prev_x = 0.0, prev_y = 0.0, prev_th = 0.0;

std::default_random_engine generator;
std::normal_distribution<double> noise_dist(0.0, 0.001); // mean 0, stddev 0.001

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    double left = 0.0, right = 0.0;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "wheel_left_joint") left = msg->position[i];
        if (msg->name[i] == "wheel_right_joint") right = msg->position[i];
    }

    if (first_joint) {
        last_left = left;
        last_right = right;
        first_joint = false;
        return;
    }

    // Add noise to wheel positions
    double delta_left = (left - last_left) + noise_dist(generator);
    double delta_right = (right - last_right) + noise_dist(generator);

    last_left = left;
    last_right = right;

    double d_left = delta_left * wheel_radius;
    double d_right = delta_right * wheel_radius;
    double d_center = (d_left + d_right) / 2.0;
    double d_theta = (d_right - d_left) / wheel_separation;

    x += d_center * cos(th + d_theta / 2.0);
    y += d_center * sin(th + d_theta / 2.0);
    th += d_theta;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/real_odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 10, jointStateCallback);

    ros::Rate r(30.0);
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while(ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        // Compute velocities
        double dt = (current_time - last_time).toSec();
        double vx = (x - prev_x) / dt;
        double vy = (y - prev_y) / dt;
        double vth = (th - prev_th) / dt;

        // Publish odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        odom_pub.publish(odom);

        prev_x = x;
        prev_y = y;
        prev_th = th;
        last_time = current_time;
        r.sleep();
    }
    return 0;
}