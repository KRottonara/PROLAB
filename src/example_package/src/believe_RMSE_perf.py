import rospy
from std_msgs.msg import Float64
import numpy as np
import threading
import time
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import math

import message_filters

belief_data = []
ground_truth_data = []
timestamps = []

lock = threading.Lock()

def sync_callback(belief_msg, gt_msg):
    with lock:
        rospy.loginfo("Synchronized belief and ground truth messages received.")
        belief_data.append(quaternion_to_yaw(belief_msg.pose.pose.orientation))
        ground_truth_data.append(quaternion_to_yaw(gt_msg.pose.orientation))

        # belief_data.append((belief_msg.pose.pose.position.y))
        # ground_truth_data.append((gt_msg.pose.position.y))

        # Use the time of the synchronized messages
        timestamps.append(belief_msg.header.stamp.to_sec() if hasattr(belief_msg, 'header') else rospy.get_time())

        # Extract yaw from quaternion for both belief and ground truth
def quaternion_to_yaw(q):
    # q: geometry_msgs/Quaternion
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    rospy.init_node('belief_rmse_perf', anonymous=True)

    belief_sub = message_filters.Subscriber('/belief', PoseWithCovarianceStamped)
    gt_sub = message_filters.Subscriber('/ground_truth_pose', PoseStamped)

    ts = message_filters.TimeSynchronizer([belief_sub, gt_sub], 30)
    ts.registerCallback(sync_callback)

    print("Collecting data for 30 seconds...")
    start_time = time.time()
    while time.time() - start_time < 30 and not rospy.is_shutdown():
        rospy.sleep(0.01)

    with lock:
        min_len = min(len(belief_data), len(ground_truth_data))
        beliefs = np.array(belief_data[:min_len])
        truths = np.array(ground_truth_data[:min_len])
        times = np.array(timestamps[:min_len])

    if min_len == 0:
        print("No data collected.")
        return

    rmse = np.sqrt(np.mean((beliefs - truths) ** 2))
    print(f"RMSE over 30s: {rmse:.4f}")

    plt.figure()
    plt.plot(times - times[0], beliefs, label='Belief')
    plt.plot(times - times[0], truths, label='Ground Truth')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Belief vs Ground Truth')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()