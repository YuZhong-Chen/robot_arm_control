#include <cmath>
#include <iostream>
#include <string>

#include "robot_arm_control/Arm_INFO.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define DISPLAY_FREQUENCY 50

static RobotArm_INFO Goal_RobotArm_info;
static RobotArm_INFO Cur_RobotArm_info;
static bool isUpdate = false;

void RobotArmJointState_Callback(const sensor_msgs::JointState::ConstPtr &msg);
void UpdateJointState();

int main(int argc, char **argv) {
    ros::init(argc, argv, "Rviz_Arm_Position_node");
    ros::NodeHandle nh;

    ros::Publisher jointState_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Subscriber RobotArmJointState_sub = nh.subscribe("RobotArmJointState", 100, RobotArmJointState_Callback);

    ros::Rate loop_rate(DISPLAY_FREQUENCY);  // Unit : Hz

    unsigned int frame_seq = 0;

    Cur_RobotArm_info.SetJointAngle(0, 90);
    Cur_RobotArm_info.SetJointAngle(1, 180);
    Cur_RobotArm_info.SetJointAngle(2, 180);

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
        msg.position.push_back((Cur_RobotArm_info.GetJointAngle(0) - 90) * PI / 180.0);
        msg.position.push_back((Cur_RobotArm_info.GetJointAngle(1) - 180) * PI / 180.0);
        msg.position.push_back((Cur_RobotArm_info.GetJointAngle(2) - 180) * PI / 180.0);
        jointState_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void RobotArmJointState_Callback(const sensor_msgs::JointState::ConstPtr &msg) {
    bool isFinish = true;
    for (int i = 0; i < 3; i++) {
        isFinish = false;
        Goal_RobotArm_info.SetJointAngle(i, msg->position[i]);
        Goal_RobotArm_info.SetJointVelocity(i, (Cur_RobotArm_info.GetJointAngle(i) <= msg->position[i] ? 1.0 : -1.0) * msg->velocity[i] / DISPLAY_FREQUENCY);
    }

    // std::cout << "Angle :\n";
    // for (int i = 0; i < 3; i++) {
    //     std::cout << Cur_RobotArm_info.GetJointName(i) << " : " << Cur_RobotArm_info.GetJointAngle(i) << '\n';
    // }
    // std::cout << "Velocity :\n";
    // for (int i = 0; i < 3; i++) {
    //     std::cout << Cur_RobotArm_info.GetJointName(i) << " : " << Cur_RobotArm_info.GetJointVelocity(i) << '\n';
    // }
    // std::cout << "Angle :\n";
    // for (int i = 0; i < 3; i++) {
    //     std::cout << Goal_RobotArm_info.GetJointName(i) << " : " << Goal_RobotArm_info.GetJointAngle(i) << '\n';
    // }
    // std::cout << "Velocity :\n";
    // for (int i = 0; i < 3; i++) {
    //     std::cout << Goal_RobotArm_info.GetJointName(i) << " : " << Goal_RobotArm_info.GetJointVelocity(i) << '\n';
    // }

    isUpdate = !isFinish;
}

void UpdateJointState() {
    bool isFinish = true;
    for (int i = 0; i < 3; i++) {
        if (Goal_RobotArm_info.GetJointVelocity(i) > 0) {
            if (Cur_RobotArm_info.GetJointAngle(i) < Goal_RobotArm_info.GetJointAngle(i)) {
                isFinish = false;
                Cur_RobotArm_info.SetJointAngle(i, Cur_RobotArm_info.GetJointAngle(i) + Goal_RobotArm_info.GetJointVelocity(i));
            } else {
                Cur_RobotArm_info.SetJointAngle(i, Goal_RobotArm_info.GetJointAngle(i));
            }
        } else {
            if (Cur_RobotArm_info.GetJointAngle(i) > Goal_RobotArm_info.GetJointAngle(i)) {
                isFinish = false;
                Cur_RobotArm_info.SetJointAngle(i, Cur_RobotArm_info.GetJointAngle(i) + Goal_RobotArm_info.GetJointVelocity(i));
            } else {
                Cur_RobotArm_info.SetJointAngle(i, Goal_RobotArm_info.GetJointAngle(i));
            }
        }
    }
    isUpdate = !isFinish;
}