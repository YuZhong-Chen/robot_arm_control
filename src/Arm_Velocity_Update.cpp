#include <iostream>
#include <string>

#include "geometry_msgs/Vector3.h"
#include "robot_arm_control/Arm_INFO.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define VEL_UPDATE_FREQUENCY 10

static RobotArm_INFO RobotArm_info;
static geometry_msgs::Vector3 EndEffectorVel;

void RobotArmControl_Vel_Callback(const geometry_msgs::Vector3::ConstPtr &msg);
void UpdateRobotArmVel();

int main(int argc, char **argv) {
    ros::init(argc, argv, "Arm_Velocity_Update");
    ros::NodeHandle nh;

    ros::Publisher JointState_pub = nh.advertise<sensor_msgs::JointState>("RobotArmJointState", 100);
    ros::Subscriber RobotArmControl_Vel_sub = nh.subscribe("RobotArmControl_Vel", 100, RobotArmControl_Vel_Callback);

    ros::Rate loop_rate(VEL_UPDATE_FREQUENCY);

    RobotArm_info.SetCurrentEndEffectorPosition(0.0, 0.0, 21.0 + 17.0 + 12.3 - 0.01);
    EndEffectorVel.x = EndEffectorVel.y = EndEffectorVel.z = 0.0;

    while (nh.ok()) {
        UpdateRobotArmVel();

        sensor_msgs::JointState msg;
        for (int i = 0; i < 3; i++) {
            msg.name.push_back(RobotArm_info.GetJointName(i));
            msg.position.push_back(RobotArm_info.GetGoalJointAngle(i));
            msg.velocity.push_back(RobotArm_info.GetJointVelocity(i));
            RobotArm_info.SetCurrentJointAngle(i, RobotArm_info.GetGoalJointAngle(i));
            RobotArm_info.SetCurrentEndEffectorPosition(RobotArm_info.GetCurrentEndEffectorPosition(0) + EndEffectorVel.x, RobotArm_info.GetCurrentEndEffectorPosition(1) + EndEffectorVel.y, RobotArm_info.GetCurrentEndEffectorPosition(2) + EndEffectorVel.z);
        }
        JointState_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void RobotArmControl_Vel_Callback(const geometry_msgs::Vector3::ConstPtr &msg) {
    EndEffectorVel.x = msg->x / VEL_UPDATE_FREQUENCY;
    EndEffectorVel.y = msg->y / VEL_UPDATE_FREQUENCY;
    EndEffectorVel.z = msg->z / VEL_UPDATE_FREQUENCY;
}

void UpdateRobotArmVel() {
    if (RobotArm_info.SetGoalEndEffectorPosition(RobotArm_info.GetCurrentEndEffectorPosition(0) + EndEffectorVel.x, RobotArm_info.GetCurrentEndEffectorPosition(1) + EndEffectorVel.y, RobotArm_info.GetCurrentEndEffectorPosition(2) + EndEffectorVel.z)) {
        for (int i = 0; i < 3; i++) {
            RobotArm_info.SetJointVelocity(i, (RobotArm_info.GetCurrentJointAngle(i) - RobotArm_info.GetGoalJointAngle(i)) * VEL_UPDATE_FREQUENCY);
        }
    } else {
        EndEffectorVel.x = EndEffectorVel.y = EndEffectorVel.z = 0.0;
    }

    std::cout << "Position : \n";
    std::cout << '\t' << "X : " << RobotArm_info.GetGoalEndEffectorPosition(0) << "\n";
    std::cout << '\t' << "Y : " << RobotArm_info.GetGoalEndEffectorPosition(1) << "\n";
    std::cout << '\t' << "Z : " << RobotArm_info.GetGoalEndEffectorPosition(2) << "\n";
}