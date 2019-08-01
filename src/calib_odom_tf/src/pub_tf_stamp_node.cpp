#include <iostream>

#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

ros::Subscriber sub;
ros::Publisher pub;

void callBack(const tf2_msgs::TFMessageConstPtr& msg);

int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_tf_stramp_rjp");
    
    ROS_INFO("start node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string topic_in, topic_out;
    pnh.param("topic_in", topic_in, std::string(""));
    pnh.param("topic_out", topic_out, std::string(""));
    ROS_INFO_STREAM("topic_in is: \t" << topic_in);
    ROS_INFO_STREAM("topic_out is: \t" << topic_out);

    ROS_INFO("setup publisher");
    pub = nh.advertise<geometry_msgs::TransformStamped>(topic_out, 1);
    ROS_INFO("setup subscriber");
    sub = nh.subscribe(topic_in, 1, &callBack);

    ros::spin();

    return 0;
}

void callBack(const tf2_msgs::TFMessageConstPtr& msg) {
    // ROS_INFO("callback is running");
    if (msg->transforms.size() == 1) {
        auto tf_stp = msg->transforms[0];
        pub.publish(tf_stp);
        ROS_INFO("publishing ...");
    } else {
        ROS_INFO("the length is not 1");
    }
}