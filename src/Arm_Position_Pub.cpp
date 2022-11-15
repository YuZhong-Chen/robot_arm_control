#include <iostream>
#include <string>

// #include "geometry_msgs/Point.h"
#include "robot_arm_control/Arm_INFO.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Arm_Position_Pub");
    ros::NodeHandle nh;
    ros::Publisher JointState_pub = nh.advertise<sensor_msgs::JointState>("RobotArmJointState", 100);
    // ros::Publisher ArmPoint_pub = nh.advertise<geometry_msgs::Point>("RobotArmPoint", 100);

    std::string command;
    int num;
    double input[3];

    while (std::cin >> command) {
        if (command == "quit" || command == "q") {
            break;
        } else if (command == "show" || command == "s") {
            std::cout << "Angle :\n";
            for (int i = 0; i < 4; i++) {
                std::cout << '\t' << RobotArm.GetJointName(i) << " : " << RobotArm.GetGoalJointAngle(i) << '\n';
            }
            std::cout << "Velocity :\n";
            for (int i = 0; i < 4; i++) {
                std::cout << '\t' << RobotArm.GetJointName(i) << " : " << RobotArm.GetJointVelocity(i) << '\n';
            }
            std::cout << "Position : \n";
            std::cout << '\t' << "X : " << RobotArm.GetGoalEndEffectorPosition(0) << "\n";
            std::cout << '\t' << "Y : " << RobotArm.GetGoalEndEffectorPosition(1) << "\n";
            std::cout << '\t' << "Z : " << RobotArm.GetGoalEndEffectorPosition(2) << "\n";

            for (int i = 0; i < 3; i++) {
                RobotArm.SetCurrentJointAngle(i, RobotArm.GetGoalJointAngle(i));
            }
            RobotArm.ForwardKinematics();
        } else if (command == "write" || command == "w") {
            std::cin >> command;
            if (command == "ang" || command == "a") {
                std::cin >> num >> input[0];
                if (!RobotArm.isJointAngleLegal(num, input[0])) {
                    std::cout << "Wrong Angle!!!\n";
                } else {
                    RobotArm.SetGoalJointAngle(num, input[0]);
                }
            } else if (command == "vel" || command == "v") {
                std::cin >> num >> input[0];
                if (!RobotArm.SetJointVelocity(num, input[0])) {
                    std::cout << "Wrong Velocity !!!\n";
                }
            } else if (command == "both" || command == "b") {
                std::cin >> num >> input[0] >> input[1];
                if (!RobotArm.isJointAngleLegal(num, input[0])) {
                    std::cout << "Wrong Angle!!!\n";
                } else {
                    RobotArm.SetGoalJointAngle(num, input[0]);
                }
                if (!RobotArm.SetJointVelocity(num, input[1])) {
                    std::cout << "Wrong Velocity !!!\n";
                }
            } else if (command == "pos" || command == "p") {
                std::cin >> input[0] >> input[1] >> input[2];
                if (!RobotArm.BackwardKinematics(input[0], input[1], input[2], true)) {
                    std::cout << "Wrong Position\n";
                }
            }
        } else if (command == "pub" || command == "p") {
            std::cin >> command;
            if (command == "state" || command == "s") {
                sensor_msgs::JointState msg;
                for (int i = 0; i < 4; i++) {
                    msg.name.push_back(RobotArm.GetJointName(i));
                    msg.position.push_back(RobotArm.GetGoalJointAngle(i));
                    msg.velocity.push_back(RobotArm.GetJointVelocity(i));
                }
                JointState_pub.publish(msg);
            }
            // else if (command == "point" || command == "p") {
            //     geometry_msgs::Point msg;
            //     msg.x = RobotArm.GetGoalEndEffectorPosition(0);
            //     msg.y = RobotArm.GetGoalEndEffectorPosition(1);
            //     msg.z = RobotArm.GetGoalEndEffectorPosition(2);
            //     ArmPoint_pub.publish(msg);
            // }
        }
    }

    return 0;
}