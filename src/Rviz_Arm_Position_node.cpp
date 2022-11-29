#include <cmath>
#include <iostream>
#include <string>

#include "robot_arm_control/Arm_INFO.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"

static int DISPLAY_FREQUENCY;

static RobotArm_INFO Goal_RobotArm_info;
static RobotArm_INFO Cur_RobotArm_info;
static bool isUpdate = false;

static std_msgs::Bool RobotAtmState_msg;
static bool isPubRobotArmState;

void RobotArmJointState_Callback(const sensor_msgs::JointState::ConstPtr &msg);
void UpdateJointState();

int main(int argc, char **argv) {
    ros::init(argc, argv, "Rviz_Arm_Position_node");
    ros::NodeHandle nh;

    ros::Publisher jointState_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher RobotArmState_pub = nh.advertise<std_msgs::Bool>("RobotArm_State", 1);
    ros::Subscriber RobotArmJointState_sub = nh.subscribe("RobotArmJointState", 1, RobotArmJointState_Callback);

    if (!nh.getParam("/Rviz_Arm_Position_node/PubRobotArmState", isPubRobotArmState)) {
        isPubRobotArmState = false;
    }
    if (!nh.getParam("/Rviz_Arm_Position_node/Frequency", DISPLAY_FREQUENCY)) {
        DISPLAY_FREQUENCY = 50;
    }

    ros::Rate loop_rate(DISPLAY_FREQUENCY);  // Unit : Hz
    unsigned int frame_seq = 0;

    Cur_RobotArm_info.SetCurrentJointAngle(0, 90.0);
    Cur_RobotArm_info.SetCurrentJointAngle(1, 289.0);
    Cur_RobotArm_info.SetCurrentJointAngle(2, 90.0);
    Cur_RobotArm_info.SetCurrentJointAngle(3, 110.0);

    while (nh.ok()) {
        if (isUpdate) {
            UpdateJointState();
        }

        sensor_msgs::JointState msg;
        msg.header.seq = ++frame_seq;
        msg.header.stamp = ros::Time::now();
        for (int i = 0; i < 3; i++) {
            msg.name.push_back(Cur_RobotArm_info.GetJointName(i));
        }
        msg.name.push_back(Cur_RobotArm_info.GetJointName(3) + "_left");
        msg.name.push_back(Cur_RobotArm_info.GetJointName(3) + "_right");
        msg.position.push_back((Cur_RobotArm_info.GetCurrentJointAngle(0)) * PI / 180.0);
        msg.position.push_back((180 - Cur_RobotArm_info.GetCurrentJointAngle(1)) * PI / 180.0);
        msg.position.push_back((180 - Cur_RobotArm_info.GetCurrentJointAngle(2)) * PI / 180.0);
        msg.position.push_back((Cur_RobotArm_info.GetCurrentJointAngle(3)) * PI / 180.0);
        msg.position.push_back(((-1 * Cur_RobotArm_info.GetCurrentJointAngle(3))) * PI / 180.0);
        jointState_pub.publish(msg);

        if (isPubRobotArmState) {
            RobotArmState_pub.publish(RobotAtmState_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void RobotArmJointState_Callback(const sensor_msgs::JointState::ConstPtr &msg) {
    bool isFinish = true;
    for (int i = 0; i < 4; i++) {
        isFinish = false;
        Goal_RobotArm_info.SetCurrentJointAngle(i, msg->position[i]);
        Goal_RobotArm_info.SetJointVelocity(i, (Cur_RobotArm_info.GetCurrentJointAngle(i) <= msg->position[i] ? 1.0 : -1.0) * fabs(msg->velocity[i]) / DISPLAY_FREQUENCY);
    }
    isUpdate = !isFinish;
    RobotAtmState_msg.data = isUpdate;
}

void UpdateJointState() {
    bool isFinish = true;
    for (int i = 0; i < 4; i++) {
        if (Goal_RobotArm_info.GetJointVelocity(i) > 0) {
            if (Cur_RobotArm_info.GetCurrentJointAngle(i) < Goal_RobotArm_info.GetCurrentJointAngle(i)) {
                isFinish = false;
                Cur_RobotArm_info.SetCurrentJointAngle(i, Cur_RobotArm_info.GetCurrentJointAngle(i) + Goal_RobotArm_info.GetJointVelocity(i));
            } else {
                Cur_RobotArm_info.SetCurrentJointAngle(i, Goal_RobotArm_info.GetCurrentJointAngle(i));
            }
        } else {
            if (Cur_RobotArm_info.GetCurrentJointAngle(i) > Goal_RobotArm_info.GetCurrentJointAngle(i)) {
                isFinish = false;
                Cur_RobotArm_info.SetCurrentJointAngle(i, Cur_RobotArm_info.GetCurrentJointAngle(i) + Goal_RobotArm_info.GetJointVelocity(i));
            } else {
                Cur_RobotArm_info.SetCurrentJointAngle(i, Goal_RobotArm_info.GetCurrentJointAngle(i));
            }
        }
    }
    isUpdate = !isFinish;
    RobotAtmState_msg.data = isUpdate;
}