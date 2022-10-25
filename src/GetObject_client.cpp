#include "robot_arm_control/GetObject.h"
#include "ros/ros.h"

static int x = 0;
static int y = 10;
static int z = 20;

int main(int argc, char **argv) {
    ros::init(argc, argv, "GetObject_client");

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<robot_arm_control::GetObject>("GetObject_service");

    robot_arm_control::GetObject srv;

    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;

    if (client.call(srv)) {
        ROS_INFO("response: isLegel:%d", srv.response.isLegal);
    } else {
        ROS_ERROR("Failed to call service test_srv");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}