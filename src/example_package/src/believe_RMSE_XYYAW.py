import rospy
import numpy as np
import threading
import time
import matplotlib.pyplot as plt
import os
import argparse
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import math
import message_filters

belief_data = {'yaw': [], 'x': [], 'y': []}
ground_truth_data = {'yaw': [], 'x': [], 'y': []}
timestamps = []

lock = threading.Lock()

def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def sync_callback(belief_msg, gt_msg):
    with lock:
        # Yaw
        belief_yaw = quaternion_to_yaw(belief_msg.pose.pose.orientation)
        gt_yaw = quaternion_to_yaw(gt_msg.pose.orientation)
        belief_data['yaw'].append(belief_yaw)
        ground_truth_data['yaw'].append(gt_yaw)
        # X
        belief_data['x'].append(belief_msg.pose.pose.position.x)
        ground_truth_data['x'].append(gt_msg.pose.position.x)
        # Y
        belief_data['y'].append(belief_msg.pose.pose.position.y)
        ground_truth_data['y'].append(gt_msg.pose.position.y)
        # Timestamp
        timestamps.append(belief_msg.header.stamp.to_sec() if hasattr(belief_msg, 'header') else rospy.get_time())

def compute_rmse(a, b):
    a = np.array(a)
    b = np.array(b)
    return np.sqrt(np.mean((a - b) ** 2))

def save_results(prefix, outdir, times, results, rmse_dict):
    var_map = {'y': 'Y', 'x': 'X', 'yaw': 'YAW'}
    label_map = {'y': 'Y [m]', 'x': 'X [m]', 'yaw': 'Yaw [rad]'}
    for key in ['y', 'x', 'yaw']:
        # Save RMSE to txt
        txt_path = os.path.join(outdir, f"{prefix}{var_map[key]}.txt")
        with open(txt_path, 'w') as f:
            f.write(f"{var_map[key]} RMSE: {rmse_dict[key]:.6f}\n")
        # Save plot to png
        plt.figure()
        plt.plot(times - times[0], results[key]['belief'], label='Belief')
        plt.plot(times - times[0], results[key]['truth'], label='Ground Truth')
        plt.xlabel('Time (s)')
        plt.ylabel(label_map[key])
        plt.title(f'{label_map[key]} - RMSE: {rmse_dict[key]:.4f}')
        plt.legend()
        plt.tight_layout()
        png_path = os.path.join(outdir, f"{prefix}{var_map[key]}.png")
        plt.savefig(png_path)
        plt.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--prefix', type=str, default='', help='Prefix for output files')
    parser.add_argument('--outdir', type=str, default='results', help='Output directory')
    args = parser.parse_args()

    prefix = args.prefix
    outdir = args.outdir

    # Print info if using defaults
    if prefix == '' or outdir == 'results':
        print("\n[INFO] Usage: python believe_RMSE_XYYAW.py --prefix=PREFIX_ --outdir=OUTPUT_DIR")
        print("[INFO] No prefix or output directory specified. Using defaults:")
        print(f"       Prefix: '{prefix}'")
        print(f"       Output directory: '{outdir}'")
        print("       Output files will be named like PREFIXY.txt, PREFIXY.png, PREFIXX.txt, etc. in the output directory.\n")

        exit(0)    

    else:
        os.makedirs(outdir, exist_ok=True)

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
            min_len = min(len(belief_data['yaw']), len(ground_truth_data['yaw']), len(timestamps))
            if min_len == 0:
                print("No data collected.")
                return
            times = np.array(timestamps[:min_len])
            results = {}
            for key in ['yaw', 'x', 'y']:
                results[key] = {
                    'belief': np.array(belief_data[key][:min_len]),
                    'truth': np.array(ground_truth_data[key][:min_len])
                }

        # Compute RMSEs
        rmse_dict = {}
        for key in ['yaw', 'x', 'y']:
            rmse_dict[key] = compute_rmse(results[key]['belief'], results[key]['truth'])

        # Save results
        save_results(prefix, outdir, times, results, rmse_dict)
        print(f"Saved plots and RMSEs to: {os.path.abspath(outdir)}")

if __name__ == '__main__':
    main()