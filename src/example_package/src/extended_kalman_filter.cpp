#include "example_package/extended_kalman_filter.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <example_package/ExtendedKalmanFilterConfig.h>
#include <cmath>

ExtendedKalmanFilter::ExtendedKalmanFilter(ros::NodeHandle &nh) : FilterNode(nh)
{
    dr_cb_ = boost::bind(&ExtendedKalmanFilter::reconfigCallback, this, _1, _2);
    dr_srv_.setCallback(dr_cb_);
}

ExtendedKalmanFilter::~ExtendedKalmanFilter() = default;

void ExtendedKalmanFilter::prediction() 
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

    // Predict state using the motion model
    _pred_mu_t1 = g(_u_t1, _mu_t0);

    // Jacobian of motion model 
    _G.setIdentity();
    _G(0, 2) = -_u_t1(0) * _dt * std::sin(_mu_t0(2));
    _G(1, 2) =  _u_t1(0) * _dt * std::cos(_mu_t0(2));

    // Predict covariance
    _pred_Cov_t1 = _G * _Cov_t0 * _G.transpose() + _R;


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

void ExtendedKalmanFilter::correction() 
{
    // Kalman gain
    _K = _pred_Cov_t1 * _H.transpose() * (_H * _pred_Cov_t1 * _H.transpose() + _Q).inverse();

    // Update state and covariance
    _mu_t1 = _pred_mu_t1 + _K * (_z_t1 - _pred_mu_t1);
    _Cov_t1 = (_I - _K * _H) * _pred_Cov_t1;

    // Prepare for next iteration
    _mu_t0 = _mu_t1;
    _Cov_t0 = _Cov_t1;

    // Publish belief
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

void ExtendedKalmanFilter::convertSensorData(const nav_msgs::Odometry::ConstPtr &odom_msg,
                                           const sensor_msgs::Imu::ConstPtr &imu_msg,
                                           const geometry_msgs::PoseStamped::ConstPtr &laser_scan_msg) 
{
    static double odom_yaw = 0.0;
    odom_yaw = tf2::getYaw(odom_msg->pose.pose.orientation);

    _u_t1 = odom_msg->twist.twist.linear.x * Eigen::Vector2d::UnitX() +
            odom_msg->twist.twist.angular.z * Eigen::Vector2d::UnitY();
             

    _z_t1(0) = laser_scan_msg->pose.position.x;
    _z_t1(1) = laser_scan_msg->pose.position.y;
    _z_t1(2) = tf2::getYaw(laser_scan_msg->pose.orientation);
}

Eigen::Vector3d ExtendedKalmanFilter::g(Eigen::Vector2d u_t1, Eigen::Vector3d mu_t0)
{
    double theta = mu_t0(2);
    double v = u_t1(0); // linear velocity
    double w = u_t1(1); // angular velocity

    Eigen::Vector3d pred_mu_t1;
    pred_mu_t1 << 
        mu_t0(0) + v * _dt * std::cos(theta), 
        mu_t0(1) + v * _dt * std::sin(theta), 
        mu_t0(2) + w * _dt;
    return pred_mu_t1;
}

void ExtendedKalmanFilter::reconfigCallback(example_package::ExtendedKalmanFilterConfig &config, uint32_t level)
{
    _R(0,0) = config.R_xx;
    _R(1,1) = config.R_yy;
    _R(2,2) = config.R_tt;
    _Q(0,0) = config.Q_xx;
    _Q(1,1) = config.Q_yy;
    _Q(2,2) = config.Q_tt;
}