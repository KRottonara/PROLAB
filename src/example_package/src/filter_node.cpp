#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/utils.h>
#include <cmath>

class FilterNode
{
public:
    FilterNode(ros::NodeHandle &nh)
    {
        odom_sub_.subscribe(nh, "/odom", 20);
        imu_sub_.subscribe(nh, "/imu", 20);
        laser_scan_sub_.subscribe(nh, "/laser_scan_matcher/pose_stamped", 20);

        sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(30), odom_sub_, imu_sub_, laser_scan_sub_)); // Increased buffer for sufficient synchronization
        sync_->setMaxIntervalDuration(ros::Duration(0.01));                                                                   // Allow up to 0.5s difference (adjust as needed)
        sync_->registerCallback(boost::bind(&FilterNode::sensorCallback, this, _1, _2, _3));

        pred_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/prediction", 10);
        bel_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/belief", 10);
    }

    virtual void sensorCallback(
        const nav_msgs::Odometry::ConstPtr &odom_msg,
        const sensor_msgs::Imu::ConstPtr &imu_msg,
        const geometry_msgs::PoseStamped::ConstPtr &laser_scan_msg)
    {
        ROS_INFO_STREAM("FilterNode: Received sensor data");
        /*
            Include your pipeline here:
            - Convert the sensor data
            - Predict the state
            - Correct (Update) the state
            - Publish the prediction (PoseWithCovarianceStamped)
        */
    }
    virtual void prediction()
    {
        ROS_INFO_STREAM("FilterNode: Prediction method called");
    }
    virtual void correction()
    {
        ROS_INFO_STREAM("FilterNode: Correction method called");
    }

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> laser_scan_sub_; // Changed from amcl_sub_

    // Define the policy type
    typedef message_filters::sync_policies::ApproximateTime<
        nav_msgs::Odometry, sensor_msgs::Imu, geometry_msgs::PoseStamped> // Changed from PoseWithCovarianceStamped
        MySyncPolicy;

    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    ros::Publisher pred_pub_;
    ros::Publisher bel_pub_;
    ros::Time last_time_;

private:
    // You can add your filter objects here
    // example for A:
    Eigen::MatrixXd A_ = Eigen::MatrixXd::Identity(6, 1);
};

class KalmanFilter : public FilterNode
{
public:
    KalmanFilter(ros::NodeHandle &nh) : FilterNode(nh), _time_t0(0) {}

private:
    // Override the sensorCallback method
    void sensorCallback(const nav_msgs::Odometry::ConstPtr &odom_msg,
                        const sensor_msgs::Imu::ConstPtr &imu_msg,
                        const geometry_msgs::PoseStamped::ConstPtr &laser_scan_msg) // Changed from PoseWithCovarianceStamped
    {
        _time_t1 = odom_msg->header.stamp;
        dt = (_time_t1 - _time_t0).toSec();
        _time_t0 = _time_t1;

        convertSensorData(odom_msg, imu_msg, laser_scan_msg);
        prediction();
    }

    void prediction() override
    {
        // ROS_INFO_STREAM("Kalman Filter: Prediction method called");
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "odom";

        // Timer starts earlier than the first callback, so we need to handle the first run case
        static bool first_run = true;
        if (first_run)
        {
            dt = 0;
            first_run = false;
        }

        _B = Eigen::Matrix3d::Identity(3, 3) * dt;
        _pred_mu_t1 = _A * _mu_t0 + _B * _u_t1;

        /*
        Empirically measured covariance matrix for the prediction step:
            [ 1.04490316e-05 -3.89057204e-06  3.60840176e-05]
            [-3.89057204e-06  5.70461249e-06 -1.53585014e-05]
            [ 3.60840176e-05 -1.53585014e-05  1.75726829e-03]
        */

        _pred_Cov_t1 = _A * _Cov_t0 * _A.transpose() + _R;

        msg.pose.pose.position.x = _mu_t1(0);
        msg.pose.pose.position.y = _mu_t1(1);
        msg.pose.pose.position.z = 0.0; // Assuming a 2D plane

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, _mu_t1(2)); // Assuming yaw is the third element
        msg.pose.pose.orientation = tf2::toMsg(q);

        // Fill the covariance matrix
        // main diagonal:
        msg.pose.covariance[0] = _pred_Cov_t1(0, 0);  // x-x
        msg.pose.covariance[7] = _pred_Cov_t1(1, 1);  // y-y
        msg.pose.covariance[35] = _pred_Cov_t1(2, 2); // yaw-yaw
        // off-diagonal:
        msg.pose.covariance[1] = _pred_Cov_t1(0, 1);  // x-y
        msg.pose.covariance[5] = _pred_Cov_t1(0, 2);  // x-yaw
        msg.pose.covariance[6] = _pred_Cov_t1(1, 0);  // y-x
        msg.pose.covariance[11] = _pred_Cov_t1(1, 2); // y-yaw
        msg.pose.covariance[30] = _pred_Cov_t1(2, 0); // yaw-x
        msg.pose.covariance[31] = _pred_Cov_t1(2, 1); // yaw-y

        ROS_INFO_STREAM("Prediction covariance: " << _Cov_t1);

        pred_pub_.publish(msg);

        correction(); // Call the correction method after prediction
    }
    void correction() override
    {
        _K = _pred_Cov_t1 * _C.transpose() * (_C * _pred_Cov_t1 * _C.transpose() + _Q).inverse();
        _mu_t1 = _pred_mu_t1 + _K * (_z_t1 - _C * _pred_mu_t1);
        _Cov_t1 = (_I - _K * _C) * _pred_Cov_t1;
        _mu_t0 = _mu_t1; // Update the state for the next prediction
        _Cov_t0 = _Cov_t1; // Update the covariance for the next prediction

        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "odom";
        msg.pose.pose.position.x = _mu_t1(0);
        msg.pose.pose.position.y = _mu_t1(1);
        msg.pose.pose.position.z = 0.0; // Assuming a 2D plane  
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, _mu_t1(2)); // Assuming yaw is the third element
        msg.pose.pose.orientation = tf2::toMsg(q);
        // Fill the covariance matrix
        // main diagonal:
        msg.pose.covariance[0] = _Cov_t1(0, 0);  // x-x
        msg.pose.covariance[7] = _Cov_t1(1, 1);  // y-y
        msg.pose.covariance[35] = _Cov_t1(2, 2); // yaw-yaw
        // off-diagonal:
        msg.pose.covariance[1] = _Cov_t1(0, 1);  // x-y
        msg.pose.covariance[5] = _Cov_t1(0, 2);  // x-yaw
        msg.pose.covariance[6] = _Cov_t1(1, 0);  // y-x
        msg.pose.covariance[11] = _Cov_t1(1, 2); // y-yaw
        msg.pose.covariance[30] = _Cov_t1(2, 0); // yaw-x
        msg.pose.covariance[31] = _Cov_t1(2, 1); // yaw-y

        bel_pub_.publish(msg);


    }
    void convertSensorData(const nav_msgs::Odometry::ConstPtr &odom_msg, const sensor_msgs::Imu::ConstPtr &imu_msg, const geometry_msgs::PoseStamped::ConstPtr &laser_scan_msg)
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

    // Kalman filter-specific members
    Eigen::MatrixXd _A = Eigen::Matrix3d::Identity(3, 3);
    Eigen::MatrixXd _B = Eigen::Matrix3d::Zero(3, 3);
    Eigen::MatrixXd _C = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Vector3d _mu_t0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d _u_t1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d _z_t1 = Eigen::Vector3d::Zero();
    Eigen::Matrix3d _Cov_t0 = (Eigen::Matrix3d() << 1.04490316e-05, -3.89057204e-06, 3.60840176e-05,
                               -3.89057204e-06, 5.70461249e-06, -1.53585014e-05,
                               3.60840176e-05, -1.53585014e-05, 1.75726829e-03)
                                  .finished();
    Eigen::Matrix3d _Q = (Eigen::Matrix3d() << 3.84694703e-04, 3.40030857e-05, 1.01914036e-05,
                          3.40030857e-05, 4.78394237e-04, -8.92618589e-08,
                          1.01914036e-05, -8.92618589e-08, 2.65893178e-05)
                             .finished();
    Eigen::MatrixXd _R = (Eigen::Matrix3d() << 1.04490316e-05, -3.89057204e-06, 3.60840176e-05,
                          -3.89057204e-06, 5.70461249e-06, -1.53585014e-05,
                          3.60840176e-05, -1.53585014e-05, 1.75726829e-03)
                             .finished();
    Eigen::MatrixXd _K = Eigen::Matrix3d::Zero(3, 3);
    Eigen::MatrixXd _I = Eigen::Matrix3d::Identity(3, 3);
    Eigen::Vector3d _mu_t1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d _pred_mu_t1 = Eigen::Vector3d::Zero();
    Eigen::MatrixXd _Cov_t1 = Eigen::Matrix3d::Identity(3, 3);
    Eigen::MatrixXd _pred_Cov_t1 = Eigen::Matrix3d::Identity(3, 3);
    ros::Time _time_t0;
    ros::Time _time_t1;
    double dt;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle nh("~");
    KalmanFilter node(nh);
    ros::spin();

    return 0;
}
