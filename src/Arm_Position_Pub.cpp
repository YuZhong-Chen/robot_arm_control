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
    double input;

    RobotArm_INFO RobotArm_info;

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
            std::cin >> command >> num >> input;
            if (command == "pos") {
                if (!RobotArm_info.SetJointAngle(num, (int)input)) {
                    std::cout << "Error !!!\n";
                }
            } else if (command == "vel") {
                if (!RobotArm_info.SetJointVelocity(num, input)) {
                    std::cout << "Error !!!\n";
                }
            } else if (command == "both") {
                if (!RobotArm_info.SetJointAngle(num, (int)input)) {
                    std::cout << "Error !!!\n";
                }
                std::cin >> input;
                if (!RobotArm_info.SetJointVelocity(num, input)) {
                    std::cout << "Error !!!\n";
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