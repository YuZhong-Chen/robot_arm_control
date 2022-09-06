#include <iostream>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Arm_Velocity_Pub");
    ros::NodeHandle nh;
    ros::Publisher EndEffectorVel_pub = nh.advertise<sensor_msgs::JointState>("RobotArmControl_Vel", 100);

    std::string command;
    double input[3] = {0};

    while (std::cin >> command) {
        if (command == "quit" || command == "q") {
            break;
        } else if (command == "show" || command == "s") {
            std::cout << "Velocity :\n";
            std::cout << "\tX : " << input[0] << '\n';
            std::cout << "\tY : " << input[1] << '\n';
            std::cout << "\tZ : " << input[2] << '\n';
        } else if (command == "write" || command == "w") {
            std::cin >> input[0] >> input[1] >> input[2];
        } else if (command == "pub" || command == "p") {
            sensor_msgs::JointState msg;
            for (int i = 0; i < 3; i++) {
                msg.velocity.push_back(input[i]);
            }
            msg.velocity.push_back(90.0);
            EndEffectorVel_pub.publish(msg);
        }
    }

    return 0;
}