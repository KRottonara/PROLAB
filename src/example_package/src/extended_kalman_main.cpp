#include <ros/ros.h>
#include "example_package/extended_kalman_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extended_kalman_filter_node");
    ros::NodeHandle nh("~");
    ExtendedKalmanFilter node(nh);
    ros::spin();
    return 0;
}