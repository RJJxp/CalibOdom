#include "../include/calib_odom.h"

#include <iostream>

#include <ros/ros.h>


// #include <iostream>
// #include <thread>
// #include <boost/bind.hpp>

// #include <ros/ros.h>
// #include <nav_msgs/Odometry.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/time_sequencer.h>
// #include <message_filters/time_synchronizer.h>

// #include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/Marker.h>

// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Dense>
// #include <eigen3/Eigen/Eigen>

void callback(const nav_msgs::OdometryConstPtr& odom_msg, const visualization_msgs::MarkerConstPtr& marker_msg ) {
    ROS_INFO ("fk fk fk");
    ROS_INFO_STREAM(" pose.x " << odom_msg->pose.pose.position.x);
    ROS_INFO_STREAM(" marker point x " << marker_msg->points[0].x);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "calib_odom_node");
    
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");

    ROS_INFO("start initializing the node");
    CalibOdom co(node_handle, private_node_handle);

    // ros::NodeHandle nh;
    // message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/raccoon/chassis/odometry", 1);
    // message_filters::Subscriber<visualization_msgs::Marker> traject_sub(nh, "/trajectory/marker", 1);
    // typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, visualization_msgs::Marker> SyncPolicy;
    // message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), odom_sub, traject_sub);
    // sync.registerCallback(boost::bind(&callback, _1, _2));

    // ros::spin();    

    return 0;
}