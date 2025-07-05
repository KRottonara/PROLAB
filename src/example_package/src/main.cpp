#include <ros/ros.h>
#include "example_package/kalman_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle nh("~");
    KalmanFilter node(nh);
    ros::spin();
    return 0;
}