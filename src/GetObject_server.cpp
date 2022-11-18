#include "robot_arm_control/Arm_INFO.h"
#include "robot_arm_control/GetObject.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"

#define MAX_QUERY_TIME 3

enum RobotArmState {
    IDLE = 0,
    GET_OBJECT,
    GET_OBJECTING,
    GRAB,
    GRABBING,
    PUT_OBJECT,
    PUT_OBJECTING,
    RELEASE
};

static int query_total = 0;
static bool RobotArmStatus = false;
static int Robot_Arm_State = IDLE;
static bool isGetObject = false;

static int Param_RobotArm_GrabAngle_Open;
static int Param_RobotArm_GrabAngle_Close;
static int Param_RobotArm_PutAngle[3];
static int Param_RobotArm_PutAngle_Vel[3];

static bool Callback(robot_arm_control::GetObject::Request &req, robot_arm_control::GetObject::Response &res);
static void RobotArmState_Callback(const std_msgs::Bool::ConstPtr &msg);

// ------------------------------------------------------------------
// Main Function.
// ------------------------------------------------------------------
int main(int argc, char **argv) {
    ros::init(argc, argv, "GetObject_server");

    // ------------------------------------------------------------------
    // ros Setup.
    // ------------------------------------------------------------------
    ros::NodeHandle nh;

    ros::Subscriber RobotArmState_sub = nh.subscribe("RobotArm_State", 1000, RobotArmState_Callback);
    ros::Publisher JointState_pub = nh.advertise<sensor_msgs::JointState>("RobotArmJointState", 1000);
    ros::Publisher Stowage_pub = nh.advertise<std_msgs::Bool>("StowageState", 1000);
    ros::ServiceServer Server = nh.advertiseService("GetObject_service", Callback);

    if (!nh.getParam("/GetObject_server/Param_RobotArm_GrabAngle_Open", Param_RobotArm_GrabAngle_Open)) {
        Param_RobotArm_GrabAngle_Open = 110;
    }
    if (!nh.getParam("/GetObject_server/Param_RobotArm_GrabAngle_Close", Param_RobotArm_GrabAngle_Close)) {
        Param_RobotArm_GrabAngle_Close = 130;
    }
    if (!nh.getParam("/GetObject_server/Param_RobotArm_PutAngle_0", Param_RobotArm_PutAngle[0])) {
        Param_RobotArm_PutAngle[0] = 90;
    }
    if (!nh.getParam("/GetObject_server/Param_RobotArm_PutAngle_1", Param_RobotArm_PutAngle[1])) {
        Param_RobotArm_PutAngle[1] = 190;
    }
    if (!nh.getParam("/GetObject_server/Param_RobotArm_PutAngle_2", Param_RobotArm_PutAngle[2])) {
        Param_RobotArm_PutAngle[2] = 270;
    }
    if (!nh.getParam("/GetObject_server/Param_RobotArm_PutAngle_Vel_0", Param_RobotArm_PutAngle_Vel[0])) {
        Param_RobotArm_PutAngle_Vel[0] = 5;
    }
    if (!nh.getParam("/GetObject_server/Param_RobotArm_PutAngle_Vel_1", Param_RobotArm_PutAngle_Vel[1])) {
        Param_RobotArm_PutAngle_Vel[1] = 5;
    }
    if (!nh.getParam("/GetObject_server/Param_RobotArm_PutAngle_Vel_2", Param_RobotArm_PutAngle_Vel[2])) {
        Param_RobotArm_PutAngle_Vel[2] = 5;
    }

    // ------------------------------------------------------------------
    // ros Loop.
    // ------------------------------------------------------------------
    while (nh.ok()) {
        // if (query_total == MAX_QUERY_TIME) {
        //     break;
        // }
        if (isGetObject) {
            isGetObject = false;
            Robot_Arm_State = GET_OBJECT;
        } else {
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        }

        switch (Robot_Arm_State) {
            case IDLE:
                break;
            case GET_OBJECT: {
                std_msgs::Bool Stowage_msg;
                Stowage_msg.data = false;
                Stowage_pub.publish(Stowage_msg);

                sensor_msgs::JointState msg;
                for (int i = 0; i < 4; i++) {
                    msg.name.push_back(RobotArm.GetJointName(i));
                    msg.position.push_back(RobotArm.GetGoalJointAngle(i));
                    msg.velocity.push_back(RobotArm.GetJointVelocity(i));
                }
                msg.position[3] = Param_RobotArm_GrabAngle_Open;
                msg.velocity[3] = 10;
                JointState_pub.publish(msg);
                Robot_Arm_State = GET_OBJECTING;
            } break;
            case GRAB: {
                ros::Duration(1.0).sleep();
                sensor_msgs::JointState msg;
                for (int i = 0; i < 4; i++) {
                    msg.name.push_back(RobotArm.GetJointName(i));
                    msg.position.push_back(RobotArm.GetGoalJointAngle(i));
                    msg.velocity.push_back(RobotArm.GetJointVelocity(i));
                }
                msg.position[3] = Param_RobotArm_GrabAngle_Close;
                msg.velocity[3] = 10;
                JointState_pub.publish(msg);
                Robot_Arm_State = GRABBING;
            } break;
            case GRABBING:
                Robot_Arm_State = PUT_OBJECT;
                ros::Duration(1.0).sleep();
                break;
            case PUT_OBJECT: {
                sensor_msgs::JointState msg;
                for (int i = 0; i < 3; i++) {
                    msg.name.push_back(RobotArm.GetJointName(i));
                    msg.position.push_back(Param_RobotArm_PutAngle[i]);
                    msg.velocity.push_back(Param_RobotArm_PutAngle_Vel[i]);
                }
                msg.name.push_back(RobotArm.GetJointName(3));
                msg.position.push_back(Param_RobotArm_GrabAngle_Close);
                msg.velocity.push_back(5);
                JointState_pub.publish(msg);
                Robot_Arm_State = PUT_OBJECTING;
            } break;
            case RELEASE: {
                sensor_msgs::JointState msg;
                for (int i = 0; i < 3; i++) {
                    msg.name.push_back(RobotArm.GetJointName(i));
                    msg.position.push_back(Param_RobotArm_PutAngle[i]);
                    msg.velocity.push_back(Param_RobotArm_PutAngle_Vel[i]);
                }
                msg.name.push_back(RobotArm.GetJointName(3));
                msg.position.push_back(Param_RobotArm_GrabAngle_Open);
                msg.velocity.push_back(5);
                JointState_pub.publish(msg);
                Robot_Arm_State = IDLE;
            } break;
            default:
                break;
        }

        ROS_INFO("State : %d", Robot_Arm_State);
    }

    return 0;
}

static void RobotArmState_Callback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data != RobotArmStatus && msg->data == false) {
        switch (Robot_Arm_State) {
            case IDLE:
                break;
            case GET_OBJECTING:
                Robot_Arm_State = GRAB;
                break;
            case GRABBING:
                Robot_Arm_State = PUT_OBJECT;
                break;
            case PUT_OBJECTING:
                Robot_Arm_State = RELEASE;
                break;
            default:
                break;
        }
    }
    RobotArmStatus = msg->data;
}

static bool Callback(robot_arm_control::GetObject::Request &req, robot_arm_control::GetObject::Response &res) {
    if (RobotArm.BackwardKinematics((double)req.x, (double)req.y, (double)req.z, true)) {
        res.isLegal = true;
        query_total++;
        isGetObject = true;
        // ROS_INFO("Pub: x=%d, y=%d, z=%d", (int)req.x, (int)req.y, (int)req.z);
    } else {
        res.isLegal = false;  // Or return false;
    }

    return true;
}
