#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped

def pose2d_callback(msg):
    pose_pub = rospy.Publisher('/laser_scan_matcher/pose_stamped_for_rviz', PoseStamped, queue_size=10)
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "odom"  # or "scanmatcher_odom" if you set a custom frame
    pose_stamped.pose.position.x = msg.x
    pose_stamped.pose.position.y = msg.y
    pose_stamped.pose.position.z = 0.0

    # Convert theta to quaternion
    import tf.transformations
    q = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
    pose_stamped.pose.orientation.x = q[0]
    pose_stamped.pose.orientation.y = q[1]
    pose_stamped.pose.orientation.z = q[2]
    pose_stamped.pose.orientation.w = q[3]

    pose_pub.publish(pose_stamped)

def main():
    rospy.init_node('laser_scan_matcher_publisher')
    rospy.Subscriber('/pose2D', Pose2D, pose2d_callback)
    rospy.spin()

if __name__ == '__main__':
    main()