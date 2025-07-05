#include "example_package/filter_node.h"

FilterNode::FilterNode(ros::NodeHandle &nh)
{
    odom_sub_.subscribe(nh, "/real_odom", 20);
    imu_sub_.subscribe(nh, "/imu", 20);
    laser_scan_sub_.subscribe(nh, "/laser_scan_matcher/pose_stamped", 20);

    sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(30), odom_sub_, imu_sub_, laser_scan_sub_));
    sync_->setMaxIntervalDuration(ros::Duration(0.01));
    sync_->registerCallback(boost::bind(&FilterNode::sensorCallback, this, _1, _2, _3));

    pred_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/prediction", 10);
    bel_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/belief", 10);
}

void FilterNode::sensorCallback(const nav_msgs::Odometry::ConstPtr &odom_msg,
                                const sensor_msgs::Imu::ConstPtr &imu_msg,
                                const geometry_msgs::PoseStamped::ConstPtr &laser_scan_msg)
{
    _time_t1 = odom_msg->header.stamp;
    dt = (_time_t1 - _time_t0).toSec();
    _time_t0 = _time_t1;

    convertSensorData(odom_msg, imu_msg, laser_scan_msg);
    prediction();
}

void FilterNode::convertSensorData(const nav_msgs::Odometry::ConstPtr &odom_msg,
                                   const sensor_msgs::Imu::ConstPtr &imu_msg,
                                   const geometry_msgs::PoseStamped::ConstPtr &laser_scan_msg)
{
    static double odom_yaw = 0.0;
    odom_yaw = tf2::getYaw(odom_msg->pose.pose.orientation);

    _u_t1 = (odom_msg->twist.twist.linear.x * cos(odom_yaw) -
             odom_msg->twist.twist.linear.y * sin(odom_yaw)) *
                Eigen::Vector3d::UnitX() +
            (odom_msg->twist.twist.linear.x * sin(odom_yaw) +
             odom_msg->twist.twist.linear.y * cos(odom_yaw)) *
                Eigen::Vector3d::UnitY() +
            odom_msg->twist.twist.angular.z * Eigen::Vector3d::UnitZ();

    _z_t1 = laser_scan_msg->pose.position.x * Eigen::Vector3d::UnitX() +
            laser_scan_msg->pose.position.y * Eigen::Vector3d::UnitY() +
            tf2::getYaw(laser_scan_msg->pose.orientation) * Eigen::Vector3d::UnitZ();
}

void FilterNode::prediction() {}
void FilterNode::correction() {}