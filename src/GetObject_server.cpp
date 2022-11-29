#include "robot_arm_control/Arm_INFO.h"
#include "robot_arm_control/GetObject.h"
#include "robot_arm_control/ServiceFinish.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"

enum RobotArmState {
    IDLE = 0,
    TAKE_PICTURE,
    TAKING_PICTURE,
    GET_OBJECT,
    GET_OBJECTING,
    GRAB,
    GRABBING,
    PUT_OBJECT,
    PUT_OBJECTING,
    RELEASE,
    RELEASING,
    STOP,
    STOPPING,
};

static bool RobotArmStatus = false;
static int Robot_Arm_State = IDLE;
static bool isFinish = false;
static int Last_State;

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
    // Setup ros.
    // ------------------------------------------------------------------
    ros::NodeHandle nh_Public;
    ros::NodeHandle nh_Private("~");

    ros::Subscriber RobotArmState_sub = nh_Public.subscribe("RobotArm_State", 1, RobotArmState_Callback);
    ros::Publisher JointState_pub = nh_Public.advertise<sensor_msgs::JointState>("RobotArmJointState", 1);
    ros::Publisher Stowage_pub = nh_Public.advertise<std_msgs::Bool>("StowageState", 1);
    ros::ServiceServer Server = nh_Public.advertiseService("GetObject_service", Callback);
    ros::ServiceClient Finish_Client = nh_Public.serviceClient<robot_arm_control::ServiceFinish>("RobotArm_ServiceFinish");

    nh_Private.param<int>("Param_RobotArm_GrabAngle_Open", Param_RobotArm_GrabAngle_Open, 110);
    nh_Private.param<int>("Param_RobotArm_GrabAngle_Close", Param_RobotArm_GrabAngle_Close, 125);
    nh_Private.param<int>("Param_RobotArm_PutAngle_0", Param_RobotArm_PutAngle[0], 90);
    nh_Private.param<int>("Param_RobotArm_PutAngle_1", Param_RobotArm_PutAngle[1], 190);
    nh_Private.param<int>("Param_RobotArm_PutAngle_2", Param_RobotArm_PutAngle[2], 270);
    nh_Private.param<int>("Param_RobotArm_PutAngle_Vel_0", Param_RobotArm_PutAngle_Vel[0], 5);
    nh_Private.param<int>("Param_RobotArm_PutAngle_Vel_1", Param_RobotArm_PutAngle_Vel[1], 5);
    nh_Private.param<int>("Param_RobotArm_PutAngle_Vel_2", Param_RobotArm_PutAngle_Vel[2], 5);

    // ------------------------------------------------------------------
    // ros Loop.
    // ------------------------------------------------------------------
    ros::Rate LoopFrequency(2);
    while (nh_Public.ok()) {
        switch (Robot_Arm_State) {
            case IDLE:
                break;
            case TAKE_PICTURE: {
                sensor_msgs::JointState msg;
                for (int i = 0; i < 4; i++) {
                    msg.name.push_back(RobotArm.GetJointName(i));
                    msg.velocity.push_back(RobotArm.GetJointVelocity(i));
                }
                msg.position.push_back(90);
                msg.position.push_back(140);
                msg.position.push_back(135);
                msg.position.push_back(110);
                msg.position[3] = Param_RobotArm_GrabAngle_Open;
                msg.velocity[3] = 10;
                JointState_pub.publish(msg);
                Robot_Arm_State = TAKING_PICTURE;
            } break;
            case GET_OBJECT: {
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
            case PUT_OBJECT: {
                std_msgs::Bool Stowage_msg;
                Stowage_msg.data = false;
                Stowage_pub.publish(Stowage_msg);

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
                Robot_Arm_State = RELEASING;
            } break;
            case STOP: {
                sensor_msgs::JointState msg;
                for (int i = 0; i < 4; i++) {
                    msg.name.push_back(RobotArm.GetJointName(i));
                    msg.velocity.push_back(RobotArm.GetJointVelocity(i));
                }
                msg.position.push_back(90);
                msg.position.push_back(289);
                msg.position.push_back(90);
                msg.position.push_back(110);
                msg.position[3] = Param_RobotArm_GrabAngle_Close;
                msg.velocity[3] = 10;
                JointState_pub.publish(msg);
                Robot_Arm_State = STOPPING;
            } break;
            default:
                break;
        }

        if (isFinish) {
            isFinish = false;
            robot_arm_control::ServiceFinish srv;
            for (int i = 0; i < 3; i++) {
                if (Finish_Client.call(srv)) {
                    break;
                } else {
                    ROS_ERROR("RobotArm: Can't call the RobotArm_ServiceFinish Server.");
                }
            }
        }

        ROS_INFO("State : %d", Robot_Arm_State);
        LoopFrequency.sleep();
        ros::spinOnce();
    }

    return 0;
}

static void RobotArmState_Callback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data != RobotArmStatus && msg->data == false) {
        switch (Robot_Arm_State) {
            case IDLE:
                break;
            case TAKING_PICTURE:
                Robot_Arm_State = IDLE;
                Last_State = TAKING_PICTURE;
                isFinish = true;
                break;
            case GET_OBJECTING:
                Robot_Arm_State = GRAB;
                Last_State = GET_OBJECTING;
                break;
            case GRABBING:
                Robot_Arm_State = PUT_OBJECT;
                Last_State = GRABBING;
                break;
            case PUT_OBJECTING:
                Robot_Arm_State = RELEASE;
                Last_State = PUT_OBJECTING;
                break;
            case RELEASING:
                Robot_Arm_State = IDLE;
                Last_State = RELEASING;
                isFinish = true;
                break;
            case STOPPING:
                Robot_Arm_State = IDLE;
                Last_State = STOPPING;
                isFinish = true;
                break;
            default:
                break;
        }
    }
    RobotArmStatus = msg->data;
}

static bool Callback(robot_arm_control::GetObject::Request &req, robot_arm_control::GetObject::Response &res) {
    if (req.x == 0 && req.y == 0 && req.z == 0) {
        res.isLegal = true;
        if (Last_State == STOPPING) {
            isFinish = true;
        } else {
            Robot_Arm_State = STOP;
        }
    } else if (req.x == -1 && req.y == -1 && req.z == -1) {
        res.isLegal = true;
        if (Last_State == TAKING_PICTURE) {
            isFinish = true;
        } else {
            Robot_Arm_State = TAKE_PICTURE;
        }
    } else if (RobotArm.BackwardKinematics((double)req.x, (double)req.y, (double)req.z, true)) {
        res.isLegal = true;
        Robot_Arm_State = GET_OBJECT;
        // ROS_INFO("Pub: x=%d, y=%d, z=%d", (int)req.x, (int)req.y, (int)req.z);
    } else {
        res.isLegal = false;  // Or return false;
    }

    return true;
}
