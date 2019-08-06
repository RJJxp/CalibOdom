#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

ros::Subscriber sub;
ros::Publisher pub;
void callBack(const visualization_msgs::MarkerArrayConstPtr& msg);

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_publisher");
    
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");


    ROS_INFO("getting param...");
    std::string topic_markers_in, topic_marker_out;
    private_node_handle.param("topic_markers_in", topic_markers_in, std::string(""));
    private_node_handle.param("topic_marker_out", topic_marker_out, std::string(""));
    
    ROS_INFO("get param finished");
    pub = node_handle.advertise<visualization_msgs::Marker>(topic_marker_out, 1);
    ROS_INFO("publisher initial succecssfully");
    sub = node_handle.subscribe(topic_markers_in, 1, &callBack);
    
    ros::spin();


    return 0;
}

void callBack(const visualization_msgs::MarkerArrayConstPtr& msg) {
    ROS_INFO_STREAM("publishing marker...");
    
    int markers_size = msg->markers.size();
    visualization_msgs::Marker marker = msg->markers[markers_size - 1];
    auto& pts = marker.points;
    auto pt = pts[pts.size() - 1];
    pts.clear();
    pts.push_back(pt);
    pub.publish(marker);

    // int markers_size = msg->markers.size();
    // pub.publish(msg->markers[markers_size - 1]);
}