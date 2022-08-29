#ifndef _ARM_INFO_H_
#define _ARM_INFO_H_

#include <string>
#include <utility>

#define PI 3.1415926

class RobotArm_INFO {
   public:
    RobotArm_INFO();

    std::string GetJointName(int num);
    double GetJointAngle(int num);
    double GetJointVelocity(int num);
    double GetEndEffectorPosition(int num);

    bool SetJointAngle(int num, double NewValue);
    bool SetJointVelocity(int num, double NewValue);

    // Set the End Effector Position and calculate each joint angle.
    bool SetEndEffectorPosition(double x, double y, double z);

    typedef struct _POINT {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    } POINT;

   private:
    std::string JointName[5];

    // Unit : Degree
    double JointAngle[5];

    // Unit : Degree / s
    double JointVelocity[5];

    // Unit : cm
    double ArmLinkLength[3];

    // The position of the End-Effector.
    // Unit : cm
    POINT EndEffectorPosition;

    // The Joint Min(first) and Max(second) Angle.
    std::pair<double, double> JointAngleLimit[5];

    bool isJointAngleLegal(double *angle);
};

#endif