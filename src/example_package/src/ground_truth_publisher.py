#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped

def wait_for_robot(model_name, timeout=2.0):
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < timeout and not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=0.5)
            if model_name in msg.name:
                rospy.loginfo(f"Found {model_name} in model_states.")
                return True
            else:
                rospy.logwarn_throttle(1.0, f"Waiting for {model_name} to appear in model_states...")
        except rospy.ROSException:
            pass
    rospy.logwarn(f"{model_name} not found after {timeout} seconds. Will keep listening and publish when it appears.")
    return False

def callback(msg):
    if 'turtlebot3_burger' in msg.name:
        idx = msg.name.index('turtlebot3_burger')
        pose = msg.pose[idx]
        out = PoseStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = "map"
        out.pose = pose
        pub.publish(out)

rospy.init_node('ground_truth_publisher')
pub = rospy.Publisher('/ground_truth_pose', PoseStamped, queue_size=1)

wait_for_robot('turtlebot3_burger', timeout=10.0)

rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
rospy.spin()