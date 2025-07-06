#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf.transformations import euler_from_quaternion

import matplotlib.pyplot as plt

class DistributionCalcNode:
    def __init__(self):
        rospy.init_node('distribution_calc_node')

        self.pred_sub = Subscriber('/prediction', PoseWithCovarianceStamped)
        self.gt_sub = Subscriber('/ground_truth_pose', PoseStamped)

        self.sync = ApproximateTimeSynchronizer([self.pred_sub, self.gt_sub], queue_size=200, slop=0.1)
        self.sync.registerCallback(self.callback)

        self.x_errors = []
        self.y_errors = []
        self.yaw_errors = []
        self.covariances = []

    def callback(self, pred_msg, gt_msg):
        rospy.loginfo("Received prediction and ground truth messages.")

        # Prediction
        pred_pos = pred_msg.pose.pose.position
        pred_ori = pred_msg.pose.pose.orientation
        pred_quat = [pred_ori.x, pred_ori.y, pred_ori.z, pred_ori.w]
        _, _, pred_yaw = euler_from_quaternion(pred_quat)

        # Ground truth
        gt_pos = gt_msg.pose.position
        gt_ori = gt_msg.pose.orientation
        gt_quat = [gt_ori.x, gt_ori.y, gt_ori.z, gt_ori.w]
        _, _, gt_yaw = euler_from_quaternion(gt_quat)

        # Errors
        x_error = pred_pos.x - gt_pos.x
        y_error = pred_pos.y - gt_pos.y
        yaw_error = pred_yaw - gt_yaw

        self.x_errors.append(x_error)
        self.y_errors.append(y_error)
        self.yaw_errors.append(yaw_error)

        rospy.loginfo(f"x_error: {x_error:.3f}, y_error: {y_error:.3f}, yaw_error: {yaw_error:.3f}")

    def plot_distribution(self):
        if not self.x_errors:
            print("No data to plot.")
            return

        # Calculate empirical covariance from error distributions
        errors = np.vstack((self.x_errors, self.y_errors, self.yaw_errors))
        empirical_cov = np.cov(errors)
        print("Empirical covariance matrix (x, y, yaw):")
        print(empirical_cov)

        plt.figure(figsize=(12, 4))
        plt.subplot(1, 3, 1)
        plt.hist(self.x_errors, bins=30, alpha=0.7)
        plt.title('X Error Distribution')
        plt.xlabel('x error (m)')
        plt.ylabel('Count')

        plt.subplot(1, 3, 2)
        plt.hist(self.y_errors, bins=30, alpha=0.7)
        plt.title('Y Error Distribution')
        plt.xlabel('y error (m)')

        plt.subplot(1, 3, 3)
        plt.hist(self.yaw_errors, bins=30, alpha=0.7)
        plt.title('Yaw Error Distribution')
        plt.xlabel('yaw error (rad)')

        plt.tight_layout()
        plt.show()

    def run(self):
        rospy.loginfo("DistributionCalcNode started. Listening to /prediction and /ground_truth_pose...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        self.plot_distribution()

if __name__ == '__main__':
    node = DistributionCalcNode()
    node.run()