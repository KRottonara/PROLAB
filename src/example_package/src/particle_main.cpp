#include "example_package/particle_filter.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "particle_filter_node");
    ros::NodeHandle nh;
    ParticleFilter pf(nh);
    ros::spin();
    return 0;
}