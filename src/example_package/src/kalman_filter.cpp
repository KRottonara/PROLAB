#include "example_package/kalman_filter.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <example_package/KalmanFilterConfig.h>

KalmanFilter::KalmanFilter(ros::NodeHandle &nh) : FilterNode(nh)
{
    dr_cb_ = boost::bind(&KalmanFilter::reconfigCallback, this, _1, _2);
    dr_srv_.setCallback(dr_cb_);
}

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::prediction() 
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";

    static bool first_run = true;
    if (first_run)
    {
        _dt = 0;
        first_run = false;
    }

    _B = Eigen::Matrix3d::Identity(3, 3) * _dt;
    _pred_mu_t1 = _A * _mu_t0 + _B * _u_t1;
    _pred_Cov_t1 = _A * _Cov_t0 * _A.transpose() + _R;

    msg.pose.pose.position.x = _pred_mu_t1(0);
    msg.pose.pose.position.y = _pred_mu_t1(1);
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, _pred_mu_t1(2));
    msg.pose.pose.orientation = tf2::toMsg(q);

    // Fill the covariance matrix
    msg.pose.covariance[0] = _pred_Cov_t1(0, 0);  // x-x
    msg.pose.covariance[7] = _pred_Cov_t1(1, 1);  // y-y
    msg.pose.covariance[35] = _pred_Cov_t1(2, 2); // yaw-yaw
    msg.pose.covariance[1] = _pred_Cov_t1(0, 1);  // x-y
    msg.pose.covariance[5] = _pred_Cov_t1(0, 2);  // x-yaw
    msg.pose.covariance[6] = _pred_Cov_t1(1, 0);  // y-x
    msg.pose.covariance[11] = _pred_Cov_t1(1, 2); // y-yaw
    msg.pose.covariance[30] = _pred_Cov_t1(2, 0); // yaw-x
    msg.pose.covariance[31] = _pred_Cov_t1(2, 1); // yaw-y

    pred_pub_.publish(msg);

    correction();
}

void KalmanFilter::correction() 
{
    _K = _pred_Cov_t1 * _C.transpose() * (_C * _pred_Cov_t1 * _C.transpose() + _Q).inverse();
    _mu_t1 = _pred_mu_t1 + _K * (_z_t1 - _C * _pred_mu_t1);
    _Cov_t1 = (_I - _K * _C) * _pred_Cov_t1;
    _mu_t0 = _mu_t1;
    _Cov_t0 = _Cov_t1;

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";
    msg.pose.pose.position.x = _mu_t1(0);
    msg.pose.pose.position.y = _mu_t1(1);
    msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, _mu_t1(2));
    msg.pose.pose.orientation = tf2::toMsg(q);

    msg.pose.covariance[0] = _Cov_t1(0, 0);  // x-x
    msg.pose.covariance[7] = _Cov_t1(1, 1);  // y-y
    msg.pose.covariance[35] = _Cov_t1(2, 2); // yaw-yaw
    msg.pose.covariance[1] = _Cov_t1(0, 1);  // x-y
    msg.pose.covariance[5] = _Cov_t1(0, 2);  // x-yaw
    msg.pose.covariance[6] = _Cov_t1(1, 0);  // y-x
    msg.pose.covariance[11] = _Cov_t1(1, 2); // y-yaw
    msg.pose.covariance[30] = _Cov_t1(2, 0); // yaw-x
    msg.pose.covariance[31] = _Cov_t1(2, 1); // yaw-y

    bel_pub_.publish(msg);

    // Log the filter cycle time
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration cycle_duration = current_time - last_time;
    ROS_INFO_STREAM("Filter cycle time: " << cycle_duration.toSec() << " seconds");
    last_time = current_time;
}

void KalmanFilter::convertSensorData(const nav_msgs::Odometry::ConstPtr &odom_msg,
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

void KalmanFilter::reconfigCallback(example_package::KalmanFilterConfig &config, uint32_t level)
{
    _R(0,0) = config.R_xx;
    _R(1,1) = config.R_yy;
    _R(2,2) = config.R_tt;
    _Q(0,0) = config.Q_xx;
    _Q(1,1) = config.Q_yy;
    _Q(2,2) = config.Q_tt;
}