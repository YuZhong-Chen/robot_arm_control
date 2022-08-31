#include "robot_arm_control/Arm_INFO.h"

#include <math.h>

#include <iostream>

double LawOfCosine(double a, double b, double c);

RobotArm_INFO::RobotArm_INFO() {
    JointName[0] = "arm_joint_1";
    JointName[1] = "arm_joint_2";
    JointName[2] = "arm_joint_3";
    JointName[3] = "Gripper";

    CurrentJointAngle[0] = 0;
    CurrentJointAngle[1] = 0;
    CurrentJointAngle[2] = 0;
    CurrentJointAngle[3] = 0;

    JointVelocity[0] = 20;
    JointVelocity[1] = 10;
    JointVelocity[2] = 10;
    JointVelocity[3] = 10;

    ArmLinkLength[0] = 21.0;
    ArmLinkLength[1] = 17.0;
    ArmLinkLength[2] = 12.3;

    JointAngleLimit[0].first = -30.0;
    JointAngleLimit[0].second = 210.0;
    JointAngleLimit[1].first = 10.0;
    JointAngleLimit[1].second = 330.0;
    JointAngleLimit[2].first = 10.0;
    JointAngleLimit[2].second = 330.0;
    JointAngleLimit[3].first = -30.0;
    JointAngleLimit[3].second = 180.0;
}

std::string RobotArm_INFO::GetJointName(int num) {
    return this->JointName[num];
}

double RobotArm_INFO::GetGoalJointAngle(int num) {
    return this->GoalJointAngle[num];
}

double RobotArm_INFO::GetCurrentJointAngle(int num) {
    return this->CurrentJointAngle[num];
}

double RobotArm_INFO::GetJointVelocity(int num) {
    return this->JointVelocity[num];
}

double RobotArm_INFO::GetGoalEndEffectorPosition(int num) {
    switch (num) {
        case 0:
            return GoalEndEffectorPosition.x;
        case 1:
            return GoalEndEffectorPosition.y;
        case 2:
            return GoalEndEffectorPosition.z;
        default:
            break;
    }
    return -1;
}

double RobotArm_INFO::GetCurrentEndEffectorPosition(int num) {
    switch (num) {
        case 0:
            return CurrentEndEffectorPosition.x;
        case 1:
            return CurrentEndEffectorPosition.y;
        case 2:
            return CurrentEndEffectorPosition.z;
        default:
            break;
    }
    return -1;
}

void RobotArm_INFO::SetGoalJointAngle(int num, double NewValue) {
    this->GoalJointAngle[num] = NewValue;
}

void RobotArm_INFO::SetCurrentJointAngle(int num, double NewValue) {
    this->CurrentJointAngle[num] = NewValue;
}

bool RobotArm_INFO::SetJointVelocity(int num, double NewValue) {
    if (num >= 0 && num <= 2 && NewValue != 0.0) {
        this->JointVelocity[num] = NewValue;
        return true;
    }
    return false;
}

bool RobotArm_INFO::SetGoalEndEffectorPosition(double x, double y, double z) {
    double answer[3];

    if (z < 0.0) {
        return false;  // Error
    }

    if (x > 0.0) {
        answer[0] = atan((double)y / x) * 57.29577;
    } else if (x < 0) {
        answer[0] = (atan((double)y / x) + PI) * 57.29577;
    } else {
        answer[0] = (PI / 2) * (y >= 0 ? 1 : -1) * 57.29577;
    }

    double c = sqrt(pow(x, 2) + pow(y, 2));

    double a = sqrt(pow(c, 2) + pow(z, 2));

    double b = sqrt(a * a + ArmLinkLength[0] * ArmLinkLength[0] - 2 * a * ArmLinkLength[0] * cos(PI / 2 - atan((double)z / c)));

    answer[1] = (LawOfCosine(ArmLinkLength[0], b, a) + LawOfCosine(ArmLinkLength[1], b, ArmLinkLength[2])) * 57.29577;

    answer[2] = LawOfCosine(ArmLinkLength[1], ArmLinkLength[2], b) * 57.29577;

    if (!isJointAngleLegal(1, answer[1]) || !isJointAngleLegal(2, answer[2])) {
        return false;
    }

    // out of range ( Error )
    if (!isJointAngleLegal(0, answer[0])) {
        if (answer[0] < JointAngleLimit[0].first) {
            answer[0] = 180 + answer[0];
        } else if (answer[0] > JointAngleLimit[0].second) {
            answer[0] = answer[0] - 180;
        }
        answer[1] = 360 - answer[1];
        answer[2] = 360 - answer[2];
        if (!isJointAngleLegal(0, answer[0]) || !isJointAngleLegal(1, answer[1]) || !isJointAngleLegal(2, answer[2])) {
            return false;
        }
    }

    GoalEndEffectorPosition.x = x;
    GoalEndEffectorPosition.y = y;
    GoalEndEffectorPosition.z = z;

    for (int i = 0; i < 3; i++) {
        GoalJointAngle[i] = answer[i];
    }
    return true;
}

bool RobotArm_INFO::isJointAngleLegal(int num, double angle) {
    if (isnan(angle) || angle < JointAngleLimit[num].first || angle > JointAngleLimit[num].second) {
        return false;
    } else {
        return true;
    }
}

double LawOfCosine(double a, double b, double c) {
    if (a == 0 || b == 0) {
        return -1;  // Wrong Input.
    }
    return acos((a * a + b * b - c * c) / (2 * a * b));
}