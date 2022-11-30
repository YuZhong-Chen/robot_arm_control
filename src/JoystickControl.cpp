#include <iostream>
#include <string>

#include "robot_arm_control/Arm_INFO.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define VEL_UPDATE_FREQUENCY 5.0

static sensor_msgs::JointState Joint;

int JointVelocity[4];

void RobotArmControl_Vel_Callback(const sensor_msgs::JointState::ConstPtr &msg);
void UpdateRobotArmVel();

int main(int argc, char **argv) {
    ros::init(argc, argv, "JoystickControl");
    ros::NodeHandle nh_Public;
    ros::NodeHandle nh_Private("~");

    ros::Publisher JointState_pub = nh_Public.advertise<sensor_msgs::JointState>("RobotArmJointState", 1);
    ros::Subscriber RobotArmControl_Vel_sub = nh_Public.subscribe("RobotArmControl_Vel", 1, RobotArmControl_Vel_Callback);

    nh_Private.param<int>("JointVelocity_0", JointVelocity[0], 1);
    nh_Private.param<int>("JointVelocity_1", JointVelocity[1], 1);
    nh_Private.param<int>("JointVelocity_2", JointVelocity[2], 1);
    nh_Private.param<int>("JointVelocity_Gripper", JointVelocity[3], 1);

    Joint.position.push_back(90);
    Joint.position.push_back(289);
    Joint.position.push_back(90);
    Joint.position.push_back(110);

    Joint.velocity.push_back(0);
    Joint.velocity.push_back(0);
    Joint.velocity.push_back(0);
    Joint.velocity.push_back(0);

    ros::Rate loop_rate(VEL_UPDATE_FREQUENCY);

    while (nh_Public.ok()) {
        UpdateRobotArmVel();
        JointState_pub.publish(Joint);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void RobotArmControl_Vel_Callback(const sensor_msgs::JointState::ConstPtr &msg) {
    Joint.velocity[0] = msg->velocity[2] * JointVelocity[0];
    Joint.velocity[1] = msg->velocity[1] * JointVelocity[1];
    Joint.velocity[2] = msg->velocity[0] * JointVelocity[2];
    Joint.velocity[3] = msg->velocity[3] * JointVelocity[3];
}

void UpdateRobotArmVel() {
    // Joint.position[0] = 1000 * Joint.velocity[0];
    Joint.position[1] = 1000 * Joint.velocity[1];

    Joint.position[2] += Joint.velocity[2] / VEL_UPDATE_FREQUENCY;
    if (Joint.position[2] <= 80) {
        Joint.position[2] = 80;
    } else if (Joint.position[2] > 300) {
        Joint.position[2] = 300;
    }

    Joint.position[3] += Joint.velocity[3] / VEL_UPDATE_FREQUENCY;
    if (Joint.position[3] <= 80) {
        Joint.position[3] = 80;
    } else if (Joint.position[3] > 130) {
        Joint.position[3] = 130;
    }
}