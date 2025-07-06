#include "example_package/particle_filter.h"
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <numeric>
#include <cmath>

ParticleFilter::ParticleFilter(ros::NodeHandle &nh, size_t num_particles)
    : FilterNode(nh),
      num_particles_(num_particles),
      gen_(std::random_device{}()),
      motion_noise_(0.0, 0.05), // Tune as needed
      angle_noise_(0.0, 0.1)   // Tune as needed
{
    std::uniform_real_distribution<double> dist_xy(-2.0, 2.0);
    std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

    particles_.resize(num_particles_);
    for (auto &p : particles_)
    {
        p.state(0) = dist_xy(gen_);    // x in [-8, 8]
        p.state(1) = dist_xy(gen_);    // y in [-8, 8]
        p.state(2) = dist_theta(gen_); // theta in [-pi, pi]
        p.weight = 1.0 / num_particles_;
    }

    particles_pub_ = nh.advertise<visualization_msgs::MarkerArray>("particles", 1);

    dr_cb_ = boost::bind(&ParticleFilter::reconfigCallback, this, _1, _2);
    dr_srv_.setCallback(dr_cb_);
    sigma_pos_ = 0.01; // Default values, match cfg
    sigma_theta_ = 0.05;
}

void ParticleFilter::convertSensorData(const nav_msgs::Odometry::ConstPtr &odom_msg,
                                       const sensor_msgs::Imu::ConstPtr &imu_msg,
                                       const geometry_msgs::PoseStamped::ConstPtr &laser_scan_msg)
{
    // Use odometry for control, laser for measurement
    _u_t1(0) = odom_msg->twist.twist.linear.x;
    _u_t1(1) = odom_msg->twist.twist.angular.z;

    _z_t1(0) = laser_scan_msg->pose.position.x;
    _z_t1(1) = laser_scan_msg->pose.position.y;
    _z_t1(2) = tf2::getYaw(laser_scan_msg->pose.orientation);
}

void ParticleFilter::prediction()
{
    static bool first_run = true;
    if (first_run)
    {
        _dt = 0;
        first_run = false;
    }
    // Move each particle using the motion model + noise
    for (auto &p : particles_)
    {
        double theta = p.state(2);
        double v = _u_t1(0) + motion_noise_(gen_);
        double w = _u_t1(1) + angle_noise_(gen_);

        p.state(0) += v * _dt * std::cos(theta);
        p.state(1) += v * _dt * std::sin(theta);
        p.state(2) += w * _dt;

        // // Clamp x and y to [-8, 8]
        // p.state(0) = std::max(-8.0, std::min(8.0, p.state(0)));
        // p.state(1) = std::max(-8.0, std::min(8.0, p.state(1)));
    }

    // Publish particles as a visualization marker array
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(num_particles_);
    for (size_t i = 0; i < particles_.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "particles";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = particles_[i].state(0);
        marker.pose.position.y = particles_[i].state(1);
        marker.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, particles_[i].state(2));
        marker.pose.orientation = tf2::toMsg(q);
        marker.scale.x = 0.2;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 0.7;
        marker.color.r = 0.1f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker_array.markers.push_back(marker);
    }
    particles_pub_.publish(marker_array);

    correction();
}

void ParticleFilter::correction()
{
    // Update weights using measurement likelihood (e.g., Gaussian)
    double weight_sum = 0.0;
    for (auto &p : particles_)
    {
        double dx = p.state(0) - _z_t1(0);
        double dy = p.state(1) - _z_t1(1);
        double dtheta = p.state(2) - _z_t1(2);
        double total_error = std::sqrt(dx * dx + dy * dy + dtheta * dtheta);
        p.weight = 1 / (total_error + 1e-12);
        weight_sum += p.weight;
    }
    // Normalize weights
    for (auto &p : particles_)
        p.weight /= weight_sum;

    // Resample
    resample();

    // Estimate mean pose for publishing
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &p : particles_)
        mean += p.weight * p.state;

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";
    msg.pose.pose.position.x = mean(0);
    msg.pose.pose.position.y = mean(1);
    msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, mean(2));
    msg.pose.pose.orientation = tf2::toMsg(q);
    // Covariance can be estimated from particles if desired
    bel_pub_.publish(msg);
}

void ParticleFilter::resample()
{
    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles_);

    std::uniform_real_distribution<double> dist(0.0, 1.0);

    while (new_particles.size() < num_particles_)
    {
        for (auto &p : particles_)
        {
            double r = dist(gen_);
            if (p.weight > r)
            {
                Particle new_particle;
                new_particle.state = p.state;               // Copy state
                new_particle.weight = 1.0 / num_particles_; // Reset weight
                new_particles.push_back(new_particle);
            }
            if (new_particles.size() >= num_particles_)
            {
                break;
            }
        }
    }

    particles_ = std::move(new_particles);
}

void ParticleFilter::reconfigCallback(example_package::ParticleFilterConfig &config, uint32_t level)
{
    sigma_pos_ = config.sigma_pos;
    sigma_theta_ = config.sigma_theta;
}