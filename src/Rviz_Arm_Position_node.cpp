#include <cmath>
#include <string>

#include "robot_arm_control/Arm_INFO.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define PI 3.14159

static RobotArm_INFO RobotArm_info;

void RobotArmJointState_Callback(const sensor_msgs::JointState::ConstPtr &msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "Rviz_Arm_Position_node");
    ros::NodeHandle nh;

    ros::Publisher jointState_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Subscriber RobotArmJointState_sub = nh.subscribe("RobotArmJointState", 100, RobotArmJointState_Callback);

    ros::Rate loop_rate(50);  // Unit : Hz

    unsigned int frame_seq = 0;

    while (nh.ok()) {
        sensor_msgs::JointState msg;
        msg.header.seq = ++frame_seq;
        msg.header.stamp = ros::Time::now();
        for (int i = 0; i < 3; i++) {
            msg.name.push_back(RobotArm_info.GetJointName(i));
            msg.position.push_back(RobotArm_info.GetJointAngle(i));
        }

        jointState_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void RobotArmJointState_Callback(const sensor_msgs::JointState::ConstPtr &msg) {
    RobotArm_info.SetJointAngle(0, (msg->position[0] - 90.0) * PI / 180.0);
    RobotArm_info.SetJointAngle(1, (180 - msg->position[1]) * PI / 180.0);
    RobotArm_info.SetJointAngle(2, (180 - msg->position[2]) * PI / 180.0);
}