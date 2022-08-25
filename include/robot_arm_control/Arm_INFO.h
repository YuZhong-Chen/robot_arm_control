#ifndef _ARM_INFO_H_
#define _ARM_INFO_H_

#include <string>

class RobotArm_INFO {
   public:
    RobotArm_INFO();

    std::string GetJointName(int num);
    double GetJointAngle(int num);
    double GetJointVelocity(int num);

    bool SetJointAngle(int num, int NewValue);
    bool SetJointVelocity(int num, double NewValue);

   private:
    std::string jointName[3];
    int jointAngle[3];
    double jointVelocity[3];
};

#endif