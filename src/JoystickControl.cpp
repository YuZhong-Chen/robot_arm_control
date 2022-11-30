#include <iostream>
#include <string>

#include "robot_arm_control/Arm_INFO.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define VEL_UPDATE_FREQUENCY 2.0

static sensor_msgs::JointState Joint;

void RobotArmControl_Vel_Callback(const sensor_msgs::JointState::ConstPtr &msg);
void UpdateRobotArmVel();

int main(int argc, char **argv) {
    ros::init(argc, argv, "JoystickControl");
    ros::NodeHandle nh;

    ros::Publisher JointState_pub = nh.advertise<sensor_msgs::JointState>("RobotArmJointState", 1);
    ros::Subscriber RobotArmControl_Vel_sub = nh.subscribe("RobotArmControl_Vel", 1, RobotArmControl_Vel_Callback);

    Joint.position.push_back(90);
    Joint.position.push_back(289);
    Joint.position.push_back(90);
    Joint.position.push_back(110);

    ros::Rate loop_rate(VEL_UPDATE_FREQUENCY);

    while (nh.ok()) {
        UpdateRobotArmVel();

        JointState_pub.publish(Joint);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void RobotArmControl_Vel_Callback(const sensor_msgs::JointState::ConstPtr &msg) {
    for (int i = 0; i < 4; i++) {
        Joint.velocity[i] = msg->velocity[i] / VEL_UPDATE_FREQUENCY;
    }
}

void UpdateRobotArmVel() {
}