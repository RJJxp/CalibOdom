#include <iostream>

#include <ros/ros.h>

#include "../include/calib_odom_tf.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "carlib_odom_tf_rjp");

    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");
    CalibOdomTf* cot = new CalibOdomTf(node_handle, private_node_handle);

    return 0;
}