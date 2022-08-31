#include <iostream>
#include <string>

#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Arm_Velocity_Pub");
    ros::NodeHandle nh;
    ros::Publisher EndEffectorVel_pub = nh.advertise<geometry_msgs::Vector3>("RobotArmControl_Vel", 100);

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
            geometry_msgs::Vector3 msg;
            msg.x = input[0];
            msg.y = input[1];
            msg.z = input[2];
            EndEffectorVel_pub.publish(msg);
        }
    }

    return 0;
}