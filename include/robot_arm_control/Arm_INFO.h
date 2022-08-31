#ifndef _ARM_INFO_H_
#define _ARM_INFO_H_

#include <string>
#include <utility>

#define PI 3.1415926

class RobotArm_INFO {
   public:
    RobotArm_INFO();

    std::string GetJointName(int num);
    double GetGoalJointAngle(int num);
    double GetCurrentJointAngle(int num);
    double GetJointVelocity(int num);
    double GetGoalEndEffectorPosition(int num);
    double GetCurrentEndEffectorPosition(int num);

    void SetGoalJointAngle(int num, double NewValue);
    void SetCurrentJointAngle(int num, double NewValue);
    bool SetJointVelocity(int num, double NewValue);

    // Set the End Effector Position and calculate each joint angle.
    bool SetGoalEndEffectorPosition(double x, double y, double z);

    bool isJointAngleLegal(int num, double angle);

    typedef struct _POINT {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    } POINT;

   private:
    std::string JointName[5];

    // 0 ~ 3 : Joint_1 ~ Joint_3
    // 4 : Gripper_left and Gripper_right
    // Unit : Degree
    double CurrentJointAngle[4];

    double GoalJointAngle[4];

    // Unit : Degree / s
    double JointVelocity[4];

    // Unit : cm
    double ArmLinkLength[3];

    // The current position of the End-Effector in 3D.
    // Unit : cm
    POINT CurrentEndEffectorPosition;

    // The goal position of the End-Effector in 3D.
    // Unit : cm
    POINT GoalEndEffectorPosition;

    // The Joint Min(first) and Max(second) Angle.
    std::pair<double, double> JointAngleLimit[4];
};

#endif