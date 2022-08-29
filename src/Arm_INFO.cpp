#include "robot_arm_control/Arm_INFO.h"

#include <math.h>

#include <iostream>

double LawOfCosine(double a, double b, double c);

RobotArm_INFO::RobotArm_INFO() {
    JointName[0] = "arm_joint_1";
    JointName[1] = "arm_joint_2";
    JointName[2] = "arm_joint_3";

    JointAngle[0] = 0;
    JointAngle[1] = 0;
    JointAngle[2] = 0;

    JointVelocity[0] = 20;
    JointVelocity[1] = 10;
    JointVelocity[2] = 10;

    ArmLinkLength[0] = 30.271;
    ArmLinkLength[1] = 26.9;
    ArmLinkLength[2] = 13.0;

    JointAngleLimit[0].first = -90.0;
    JointAngleLimit[0].second = 360.0;
    JointAngleLimit[1].first = 45.0;
    JointAngleLimit[1].second = 315.0;
    JointAngleLimit[2].first = 30.0;
    JointAngleLimit[2].second = 315.0;
}

std::string RobotArm_INFO::GetJointName(int num) {
    if (num >= 0 && num <= 2) {
        return this->JointName[num];
    }
    return "";
}

double RobotArm_INFO::GetJointAngle(int num) {
    if (num >= 0 && num <= 2) {
        return this->JointAngle[num];
    }
    return 0;
}

double RobotArm_INFO::GetJointVelocity(int num) {
    if (num >= 0 && num <= 2) {
        return this->JointVelocity[num];
    }
    return 0.01;
}

double RobotArm_INFO::GetEndEffectorPosition(int num) {
    switch (num) {
        case 0:
            return EndEffectorPosition.x;
        case 1:
            return EndEffectorPosition.y;
        case 2:
            return EndEffectorPosition.z;
        default:
            break;
    }
    return -1;
}

bool RobotArm_INFO::SetJointAngle(int num, double NewValue) {
    if (num >= 0 && num <= 2) {
        this->JointAngle[num] = NewValue;
        return true;
    }
    return false;
}

bool RobotArm_INFO::SetJointVelocity(int num, double NewValue) {
    if (num >= 0 && num <= 2 && NewValue != 0.0) {
        this->JointVelocity[num] = NewValue;
        return true;
    }
    return false;
}

bool RobotArm_INFO::SetEndEffectorPosition(double x, double y, double z) {
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

    // out of range ( Error )
    if (!isJointAngleLegal(answer)) {
        return false;
    }

    EndEffectorPosition.x = x;
    EndEffectorPosition.y = y;
    EndEffectorPosition.z = z;
    for (int i = 0; i < 3; i++) {
        JointAngle[i] = answer[i];
    }
    return true;
}

bool RobotArm_INFO::isJointAngleLegal(double *angle) {
    for (int i = 0; i < 3; i++) {
        if (angle[i] < JointAngleLimit[i].first || angle[i] > JointAngleLimit[i].second || isnan(angle[i])) {
            for (int j = 0; j < 3; j++) {
                std::cout << angle[j] << " ";
            }
            std::cout << '\n';
            return false;
        }
    }
    return true;
}

double LawOfCosine(double a, double b, double c) {
    if (a == 0 || b == 0) {
        return -1;  // Wrong Input.
    }
    return acos((a * a + b * b - c * c) / (2 * a * b));
}