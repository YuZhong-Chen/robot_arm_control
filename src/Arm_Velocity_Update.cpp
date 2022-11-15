#include <iostream>
#include <string>

#include "robot_arm_control/Arm_INFO.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define VEL_UPDATE_FREQUENCY 10.0

static sensor_msgs::JointState EndEffectorVel;

void RobotArmControl_Vel_Callback(const sensor_msgs::JointState::ConstPtr &msg);
void UpdateRobotArmVel();

int main(int argc, char **argv) {
    ros::init(argc, argv, "Arm_Velocity_Update");
    ros::NodeHandle nh;

    ros::Publisher JointState_pub = nh.advertise<sensor_msgs::JointState>("RobotArmJointState", 100);
    ros::Subscriber RobotArmControl_Vel_sub = nh.subscribe("RobotArmControl_Vel", 100, RobotArmControl_Vel_Callback);

    ros::Rate loop_rate(VEL_UPDATE_FREQUENCY);

    // Init Position.
    RobotArm.SetCurrentEndEffectorPosition(0.0, 17.0 + 12.3 - 0.001, 21.0 - 0.001);
    RobotArm.SetJointVelocity(3, 20);

    for (int i = 0; i < 3; i++) {
        EndEffectorVel.velocity.push_back(0.0);
    }
    RobotArm.SetGoalJointAngle(3, 90.0);  // This is Gripper angle.

    while (nh.ok()) {
        UpdateRobotArmVel();

        sensor_msgs::JointState msg;
        for (int i = 0; i < 4; i++) {
            // msg.name.push_back(RobotArm_Info.GetJointName(i));
            msg.position.push_back(RobotArm.GetGoalJointAngle(i));
            msg.velocity.push_back(RobotArm.GetJointVelocity(i));
            RobotArm.SetCurrentJointAngle(i, RobotArm.GetGoalJointAngle(i));
        }
        JointState_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void RobotArmControl_Vel_Callback(const sensor_msgs::JointState::ConstPtr &msg) {
    for (int i = 0; i < 4; i++) {
        EndEffectorVel.velocity[i] = msg->velocity[i] / VEL_UPDATE_FREQUENCY;
    }
    RobotArm.SetGoalJointAngle(3, msg->velocity[3]);  // This is Gripper angle.
}

void UpdateRobotArmVel() {
    if (RobotArm.BackwardKinematics(RobotArm.GetCurrentEndEffectorPosition(0) + EndEffectorVel.velocity[0], RobotArm.GetCurrentEndEffectorPosition(1) + EndEffectorVel.velocity[1], RobotArm.GetCurrentEndEffectorPosition(2) + EndEffectorVel.velocity[2], true)) {
        for (int i = 0; i < 3; i++) {
            RobotArm.SetJointVelocity(i, (RobotArm.GetCurrentJointAngle(i) - RobotArm.GetGoalJointAngle(i)) * VEL_UPDATE_FREQUENCY);
        }
        RobotArm.SetCurrentEndEffectorPosition(RobotArm.GetCurrentEndEffectorPosition(0) + EndEffectorVel.velocity[0], RobotArm.GetCurrentEndEffectorPosition(1) + EndEffectorVel.velocity[1], RobotArm.GetCurrentEndEffectorPosition(2) + EndEffectorVel.velocity[2]);
    }
    // else {
    //     std::cout << "Can't go to " << RobotArm.GetCurrentEndEffectorPosition(0) + EndEffectorVel.x << " " << RobotArm.GetCurrentEndEffectorPosition(1) + EndEffectorVel.y << " " << RobotArm_Info.GetCurrentEndEffectorPosition(2) + EndEffectorVel.z << "\n";
    // }

    std::cout << "Position : \n";
    std::cout << '\t' << "X : " << RobotArm.GetGoalEndEffectorPosition(0) << "\n";
    std::cout << '\t' << "Y : " << RobotArm.GetGoalEndEffectorPosition(1) << "\n";
    std::cout << '\t' << "Z : " << RobotArm.GetGoalEndEffectorPosition(2) << "\n";
}