#include "robot_arm_control/Arm_INFO.h"
#include "robot_arm_control/GetObject.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

// #define DEBUG

#define MAX_QUERY_TIME 3

static int query_total = 0;

typedef struct {
    bool isPub = false;
} PUB_DATA;

static PUB_DATA PubData;

static bool Callback(robot_arm_control::GetObject::Request &req, robot_arm_control::GetObject::Response &res) {
    if (RobotArm.BackwardKinematics((double)req.x, (double)req.y, (double)req.z, true)) {
        res.isLegal = true;
        query_total++;
        PubData.isPub = true;

#ifdef DEBUG
        ROS_INFO("Pub: x=%d, y=%d, z=%d", (int)req.x, (int)req.y, (int)req.z);
#endif

    } else {
        res.isLegal = false;  // Or return false;
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "GetObject_server");
    ros::NodeHandle nh;

    ros::Publisher JointState_pub = nh.advertise<sensor_msgs::JointState>("RobotArmJointState", 100);
    ros::ServiceServer Server = nh.advertiseService("GetObject_service", Callback);

    while (nh.ok()) {
        if (query_total >= MAX_QUERY_TIME) {
            break;
        } else if (PubData.isPub) {
            PubData.isPub = false;
            sensor_msgs::JointState msg;
            for (int i = 0; i < 4; i++) {
                msg.name.push_back(RobotArm.GetJointName(i));
                msg.position.push_back(RobotArm.GetGoalJointAngle(i));
                msg.velocity.push_back(RobotArm.GetJointVelocity(i));
            }
            JointState_pub.publish(msg);
        } else {
            ros::Duration(3.0).sleep();
            ros::spinOnce();
        }
    }

    return 0;
}