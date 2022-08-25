#include <cmath>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define PI 3.14159

int main(int argc, char **argv) {
    ros::init(argc, argv, "Rviz_Arm_Position_Pub");
    ros::NodeHandle nh;
    ros::Publisher jointState_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Rate loop_rate(10);  // Unit : Hz

    unsigned int frame_seq = 0;
    std::string jointName[3] = {"arm_joint_1", "arm_joint_2", "arm_joint_3"};
    double jointAngle[3] = {0.0, PI / 2, -PI / 2};

    while (nh.ok()) {
        sensor_msgs::JointState msg;
        msg.header.seq = ++frame_seq;
        msg.header.stamp = ros::Time::now();
        for (int i = 0; i < 3; i++) {
            msg.name.push_back(jointName[i]);
            msg.position.push_back(jointAngle[i]);
        }

        jointState_pub.publish(msg);

        jointAngle[0] += 0.01;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}