#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/utils.h>
#include <memory>

class FilterNode
{
public:
    FilterNode(ros::NodeHandle &nh);
    virtual ~FilterNode() = default;

    virtual void prediction();
    virtual void correction();
    virtual void convertSensorData(const nav_msgs::Odometry::ConstPtr &odom_msg,
                                   const sensor_msgs::Imu::ConstPtr &imu_msg,
                                   const geometry_msgs::PoseStamped::ConstPtr &laser_scan_msg);

    void sensorCallback(const nav_msgs::Odometry::ConstPtr &odom_msg,
                        const sensor_msgs::Imu::ConstPtr &imu_msg,
                        const geometry_msgs::PoseStamped::ConstPtr &laser_scan_msg);

protected:
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> laser_scan_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        nav_msgs::Odometry, sensor_msgs::Imu, geometry_msgs::PoseStamped> MySyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    ros::Publisher pred_pub_;
    ros::Publisher bel_pub_;
    Eigen::Vector3d _z_t1 = Eigen::Vector3d::Zero();
    ros::Time _time_t0;
    ros::Time _time_t1;
    double _dt = 0.0;
};