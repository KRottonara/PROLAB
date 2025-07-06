#pragma once

#include "example_package/filter_node.h"
#include <ros/ros.h>
#include <random>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <example_package/ParticleFilterConfig.h>
#include <dynamic_reconfigure/server.h>

struct Particle
{
    Eigen::Vector3d state; // [x, y, theta]
    double weight;
};

class ParticleFilter : public FilterNode
{
public:
    ParticleFilter(ros::NodeHandle &nh, size_t num_particles = 1000);
    ~ParticleFilter() override = default;

private:
    void prediction() override;
    void correction() override;
    void convertSensorData(const nav_msgs::Odometry::ConstPtr &odom_msg,
                           const sensor_msgs::Imu::ConstPtr &imu_msg,
                           const geometry_msgs::PoseStamped::ConstPtr &laser_scan_msg) override;

    void resample();
    void reconfigCallback(example_package::ParticleFilterConfig &config, uint32_t level);

    // Particle filter-specific members
    std::vector<Particle> particles_;
    size_t num_particles_;
    std::mt19937 gen_;
    std::normal_distribution<double> motion_noise_;
    std::normal_distribution<double> angle_noise_;
    Eigen::Vector2d _u_t1 = Eigen::Vector2d::Zero(); // Control input [v, w]
    Eigen::Vector3d _z_t1 = Eigen::Vector3d::Zero();
    ros::Publisher particles_pub_;
    dynamic_reconfigure::Server<example_package::ParticleFilterConfig> dr_srv_;
    dynamic_reconfigure::Server<example_package::ParticleFilterConfig>::CallbackType dr_cb_;
    double sigma_pos_;
    double sigma_theta_;
};