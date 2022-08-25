#include <iostream>
#include <string>

#include "robot_arm_control/Arm_INFO.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Arm_Position_Pub");
    ros::NodeHandle nh;
    ros::Publisher jointState_pub = nh.advertise<sensor_msgs::JointState>("RobotArmJointState", 100);

    std::string command;
    int num;
    double input[2];

    RobotArm_INFO RobotArm_info;
    RobotArm_info.SetJointAngle(0, 90);
    RobotArm_info.SetJointAngle(1, 180);
    RobotArm_info.SetJointAngle(2, 180);

    while (std::cin >> command) {
        if (command == "quit" || command == "q") {
            break;
        } else if (command == "show" || command == "s") {
            std::cout << "Angle :\n";
            for (int i = 0; i < 3; i++) {
                std::cout << RobotArm_info.GetJointName(i) << " : " << RobotArm_info.GetJointAngle(i) << '\n';
            }
            std::cout << "Velocity :\n";
            for (int i = 0; i < 3; i++) {
                std::cout << RobotArm_info.GetJointName(i) << " : " << RobotArm_info.GetJointVelocity(i) << '\n';
            }

        } else if (command == "write" || command == "w") {
            std::cin >> command >> num >> input[0];
            if (command == "pos" || command == "p") {
                if (!RobotArm_info.SetJointAngle(num, input[0])) {
                    std::cout << "Error in setting Angle!!!\n";
                }
            } else if (command == "vel" || command == "v") {
                if (!RobotArm_info.SetJointVelocity(num, input[0])) {
                    std::cout << "Error in setting Velocity !!!\n";
                }
            } else if (command == "both" || command == "b") {
                std::cin >> input[1];
                if (!RobotArm_info.SetJointAngle(num, input[0])) {
                    std::cout << "Error in setting Angle!!!\n";
                }
                if (!RobotArm_info.SetJointVelocity(num, input[1])) {
                    std::cout << "Error in setting Velocity !!!\n";
                }
            }
        } else if (command == "pub" || command == "p") {
            sensor_msgs::JointState msg;
            for (int i = 0; i < 3; i++) {
                msg.name.push_back(RobotArm_info.GetJointName(i));
                msg.position.push_back(RobotArm_info.GetJointAngle(i));
                msg.velocity.push_back(RobotArm_info.GetJointVelocity(i));
            }
            jointState_pub.publish(msg);
        }
    }

    return 0;
}