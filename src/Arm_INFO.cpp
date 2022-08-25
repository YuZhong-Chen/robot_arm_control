#include "robot_arm_control/Arm_INFO.h"

#include <cmath>

RobotArm_INFO::RobotArm_INFO() {
    jointName[0] = "arm_joint_1";
    jointName[1] = "arm_joint_2";
    jointName[2] = "arm_joint_3";

    jointAngle[0] = 0;
    jointAngle[1] = 0;
    jointAngle[2] = 0;

    jointVelocity[0] = 0.01;
    jointVelocity[1] = 0.01;
    jointVelocity[2] = 0.01;
}

std::string RobotArm_INFO::GetJointName(int num) {
    if (num >= 0 && num <= 2) {
        return this->jointName[num];
    }
    return "";
}

double RobotArm_INFO::GetJointAngle(int num) {
    if (num >= 0 && num <= 2) {
        return this->jointAngle[num];
    }
    return 0;
}

double RobotArm_INFO::GetJointVelocity(int num) {
    if (num >= 0 && num <= 2) {
        return this->jointVelocity[num];
    }
    return 0.01;
}

bool RobotArm_INFO::SetJointAngle(int num, double NewValue) {
    if (num >= 0 && num <= 2) {
        this->jointAngle[num] = NewValue;
        return true;
    }
    return false;
}

bool RobotArm_INFO::SetJointVelocity(int num, double NewValue) {
    if (num >= 0 && num <= 2 && NewValue != 0.0) {
        this->jointVelocity[num] = fabs(NewValue);
        return true;
    }
    return false;
}