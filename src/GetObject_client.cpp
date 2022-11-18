#include "robot_arm_control/GetObject.h"
#include "ros/ros.h"

static int x = 0;
static int y = 20;
static int z = 5;

int main(int argc, char **argv) {
    ros::init(argc, argv, "GetObject_client");

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<robot_arm_control::GetObject>("GetObject_service");

    if (!nh.getParam("/GetObject_client/x", x)) {
        x = 0;
    }
    if (!nh.getParam("/GetObject_client/y", y)) {
        y = 0;
    }
    if (!nh.getParam("/GetObject_client/z", z)) {
        z = 0;
    }

    robot_arm_control::GetObject srv;

    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;

    if (client.call(srv)) {
        if (srv.response.isLegal) {
            ROS_INFO("Get Object at (%d,%d,%d).", srv.request.x, srv.request.y, srv.request.z);
        } else {
            ROS_ERROR("Wrong Coordinate : (%d,%d,%d).", srv.request.x, srv.request.y, srv.request.z);
        }

    } else {
        ROS_ERROR("Failed to call service GetObject_server");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}