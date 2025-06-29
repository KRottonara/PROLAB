#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <XmlRpcValue.h>

struct Goal {
    double px, py, pz;
    double ox, oy, oz, ow;
};

bool goal_reached = false;
std::string current_goal_id;

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
    for (const auto& status : msg->status_list) {
        if (status.goal_id.id == current_goal_id &&
            status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
            goal_reached = true;
            break;
        }
    }
}

int main(int argc, char** argv) {
    ROS_INFO("Starting goal publisher node...");
    ros::init(argc, argv, "publish_goals");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
    ros::Subscriber sub = nh.subscribe("/move_base/status", 1, statusCallback);

    // Read goals from parameter server
    std::vector<Goal> goals;
    XmlRpc::XmlRpcValue goal_list;
    if (nh.getParam("goals", goal_list) && goal_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < goal_list.size(); ++i) {
            if (goal_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray && goal_list[i].size() == 7) {
                Goal g;
                g.px = static_cast<double>(goal_list[i][0]);
                g.py = static_cast<double>(goal_list[i][1]);
                g.pz = static_cast<double>(goal_list[i][2]);
                g.ox = static_cast<double>(goal_list[i][3]);
                g.oy = static_cast<double>(goal_list[i][4]);
                g.oz = static_cast<double>(goal_list[i][5]);
                g.ow = static_cast<double>(goal_list[i][6]);
                goals.push_back(g);
            }
            else {
                ROS_ERROR("Type is not XmlRpc::XmlRpcValue::TypeArray or the array length is not 7.");
            }
        }
        ROS_INFO("Loaded parameters from server.");
    } else {
        ROS_ERROR("Could not load goals from parameter server!");
        return 1;
    }

    ros::Rate rate(1);

    // Wait for move_base to subscribe
    while (pub.getNumSubscribers() == 0 && ros::ok()) {
        ROS_INFO("Waiting for move_base to subscribe to /move_base/goal...");
        ros::spinOnce();
        rate.sleep();
    }

    int goal_id = 0;
    while (ros::ok()) {
        for (size_t i = 0; i < goals.size() && ros::ok(); ++i) {
            move_base_msgs::MoveBaseActionGoal goal_msg;
            goal_msg.header.stamp = ros::Time::now();
            goal_msg.header.frame_id = "map";
            goal_msg.goal_id.id = "goal_" + std::to_string(goal_id++);
            current_goal_id = goal_msg.goal_id.id;
            goal_msg.goal.target_pose.header = goal_msg.header;
            goal_msg.goal.target_pose.pose.position.x = goals[i].px;
            goal_msg.goal.target_pose.pose.position.y = goals[i].py;
            goal_msg.goal.target_pose.pose.position.z = goals[i].pz;
            goal_msg.goal.target_pose.pose.orientation.x = goals[i].ox;
            goal_msg.goal.target_pose.pose.orientation.y = goals[i].oy;
            goal_msg.goal.target_pose.pose.orientation.z = goals[i].oz;
            goal_msg.goal.target_pose.pose.orientation.w = goals[i].ow;

            goal_reached = false;
            
            pub.publish(goal_msg);
            ROS_INFO("Published goal %d", static_cast<int>(i + 1));
            
            ros::Duration(2.0).sleep();

            // Wait until the goal is reached
            while (ros::ok() && !goal_reached) {
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("Goal %d reached", static_cast<int>(i + 1));
        }
    }
    return 0;
}