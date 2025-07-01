#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
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
        odom_sub_.subscribe(nh, "/odom", 10);
        imu_sub_.subscribe(nh, "/imu", 10);
        amcl_sub_.subscribe(nh, "/amcl_pose", 10);

        sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), odom_sub_, imu_sub_, amcl_sub_));
        sync_->registerCallback(boost::bind(&FilterNode::sensorCallback, this, _1, _2, _3));

        // sync_.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu, geometry_msgs::PoseWithCovarianceStamped>(
        //     odom_sub_, imu_sub_, amcl_sub_, 10));
        // sync_->registerCallback(boost::bind(&FilterNode::sensorCallback, this, _1, _2, _3));

        pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/prediction", 10);
    }

    virtual void sensorCallback(
        const nav_msgs::Odometry::ConstPtr &odom_msg,
        const sensor_msgs::Imu::ConstPtr &imu_msg,
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_msg)
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
        // This method can be overridden in derived classes to implement specific prediction logic
    }
    virtual void correction()
    {
        ROS_INFO_STREAM("FilterNode: Correction method called");
        // This method can be overridden in derived classes to implement specific correction logic
    }

    /*
        you can add your filter methods here
    */

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> amcl_sub_;

    // Define the policy type
    typedef message_filters::sync_policies::ApproximateTime<
        nav_msgs::Odometry, sensor_msgs::Imu, geometry_msgs::PoseWithCovarianceStamped>
        MySyncPolicy;

    // In your class:
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    // std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu, geometry_msgs::PoseWithCovarianceStamped>> sync_;

    ros::Publisher pub_;
    ros::Time last_time_;

private:
    // You can add your filter objects here
    // example for A:
    Eigen::MatrixXd A_ = Eigen::MatrixXd::Identity(6, 1);
};

class KalmanFilter : public FilterNode
{
public:
    // Inherit the constructor from FilterNode
    KalmanFilter(ros::NodeHandle &nh) : FilterNode(nh), _time_t0(0) {}

private:
    // Override the sensorCallback method
    void sensorCallback(const nav_msgs::Odometry::ConstPtr &odom_msg,
                        const sensor_msgs::Imu::ConstPtr &imu_msg,
                        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_msg)
    {
        // ROS_INFO_STREAM("Kalman Filter: Received sensor data");

        _time_t1 = odom_msg->header.stamp;
        dt = (_time_t1 - _time_t0).toSec();
        _time_t0 = _time_t1;
        // ROS_INFO_STREAM("Time step (dt): " << dt);

        ROS_INFO_STREAM("AMCL Pose Estimate: x=" << amcl_msg->pose.pose.position.x
                                                 << ", y=" << amcl_msg->pose.pose.position.y
                                                 << ", yaw=" << tf2::getYaw(amcl_msg->pose.pose.orientation));
        geometry_msgs::PoseWithCovarianceStamped prediction_msg = *amcl_msg;
        prediction_msg.header.stamp = ros::Time::now();

        pub_.publish(prediction_msg);
        convertSensorData(odom_msg, imu_msg);
        prediction();

        // Your Kalman filter implementation here
        // Convert the sensor data to the state vector
        // Predict the next state using the Kalman filter equations
        // Correct the predicted state with the new measurements
        // Publish the updated state as a PoseWithCovarianceStamped message
    }

    void prediction() override
    {
        // ROS_INFO_STREAM("Kalman Filter: Prediction method called");
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "odom";
        // Assuming _mu_t0 contains [x, y, theta]
        msg.pose.pose.position.x = _mu_t0(0);
        msg.pose.pose.position.y = _mu_t0(1);
        msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, _mu_t0(2));
        msg.pose.pose.orientation = tf2::toMsg(q);
        // Optionally set covariance here if needed
        // pub_.publish(msg);
    }
    void correction() override
    {
        // ROS_INFO_STREAM("Kalman Filter: Correction method called");
    }
    void convertSensorData(const nav_msgs::Odometry::ConstPtr &odom_msg, const sensor_msgs::Imu::ConstPtr &imu_msg)
    {
        _u_t1 = odom_msg->twist.twist.linear.x * Eigen::VectorXf::Unit(3, 0) +
                odom_msg->twist.twist.linear.y * Eigen::VectorXf::Unit(3, 1) +
                odom_msg->twist.twist.angular.z * Eigen::VectorXf::Unit(3, 2);

        // 1. Get yaw from IMU orientation
        tf2::Quaternion q(
            imu_msg->orientation.x,
            imu_msg->orientation.y,
            imu_msg->orientation.z,
            imu_msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 2. Build rotation matrix
        Eigen::Matrix2f R;
        R << cos(yaw), -sin(yaw),
            sin(yaw), cos(yaw);

        // 3. Get acceleration in base frame
        Eigen::Vector2f a_base;
        a_base << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y;

        // 4. Transform to world frame
        Eigen::Vector2f a_world = R * a_base;

        // 5. Calculate velocity in world frame
        // static float vel_offs_x = imu_msg->linear_acceleration.x * dt;
        // static float vel_offs_y = imu_msg->linear_acceleration.y * dt;
        static float vel_offs_x = a_world(0) * dt; // Convert to m/s
        static float vel_offs_y = a_world(1) * dt; // Convert to

        static float vel_x = 0.0;
        static float vel_y = 0.0;

        vel_x += a_world(0) * dt - vel_offs_x; // Convert to m/s
        vel_y += a_world(1) * dt - vel_offs_y; // Convert to m/s

        vel_offs_x = 0;
        vel_offs_y = 0;

        // static float offset_X = 9;
        // static float offset_Y = -68;
        static float offset_X = 0;
        static float offset_Y = 0;

        _z_t1 = _mu_t0 + vel_x * dt * Eigen::VectorXf::Unit(3, 0) + offset_X * Eigen::VectorXf::Unit(3, 0) +
                vel_y * dt * Eigen::VectorXf::Unit(3, 1) + offset_Y * Eigen::VectorXf::Unit(3, 1) +
                imu_msg->angular_velocity.z * dt * Eigen::VectorXf::Unit(3, 2);
        _mu_t0 = _z_t1;
        // ROS_INFO_STREAM("Linear Acceleration: " << imu_msg->linear_acceleration.x << ", " << imu_msg->linear_acceleration.y);

        offset_X = 0;
        offset_Y = 0;

        ROS_INFO_STREAM_THROTTLE(0.5, "Velocity X: " << vel_x << ", Velocity Y: " << vel_y);
    }

    // Kalman filter-specific members
    Eigen::MatrixXd _A = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd _B = Eigen::MatrixXd::Zero(6, 3);
    Eigen::MatrixXd _C = Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXf _mu_t0 = Eigen::VectorXf::Zero(3);
    Eigen::VectorXf _u_t1 = Eigen::VectorXf::Zero(3);
    Eigen::VectorXf _z_t1 = Eigen::VectorXf::Zero(6);
    Eigen::MatrixXd _Cov_t0 = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd _Q = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd _R = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd _K = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd _I = Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXf _mu_t1 = Eigen::VectorXf::Zero(3);
    Eigen::MatrixXd _Cov_t1 = Eigen::MatrixXd::Identity(6, 6);
    ros::Time _time_t0;
    ros::Time _time_t1;
    double dt; // Time step, can be adjusted based on your application
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle nh("~");
    KalmanFilter node(nh);
    ros::spin();

    return 0;
}
