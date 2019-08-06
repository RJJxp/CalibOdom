#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h> 

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

ros::Subscriber sub;
ros::Publisher pub;
// used in callBack function
Eigen::MatrixXd cal_mat;
std::string adjust_method;

void callBack(const nav_msgs::OdometryConstPtr& msg);
Eigen::MatrixXd getMatParam(std::string filepath, std::string adjust_method);


int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_result_rjp");
    
    ROS_INFO("start node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string topic_in, topic_out;
    pnh.param("topic_in", topic_in, std::string(""));
    pnh.param("topic_out", topic_out, std::string("/test/odometry"));
    ROS_INFO_STREAM("topic_in " << topic_in);
    ROS_INFO_STREAM("topic_out " << topic_out);

    std::string mat_file_path;
    pnh.param("mat_file_path", mat_file_path, std::string(""));
    pnh.param("adjust_method", adjust_method, std::string("linear"));
    ROS_INFO_STREAM("mat_file_path " << mat_file_path);
    ROS_INFO_STREAM("adjust method " << adjust_method);

    cal_mat = getMatParam(mat_file_path, adjust_method);
    ROS_INFO_STREAM("the mat is \n" << cal_mat);

    pub = nh.advertise<nav_msgs::Odometry>(topic_out, 1);
    sub = nh.subscribe(topic_in, 1, &callBack);
    
    ros::spin();

    return 0;
}

Eigen::MatrixXd getMatParam(std::string filepath, std::string adjust_method) {
    std::ifstream infile(filepath, std::ios::in);
    ROS_INFO("read mat param done");
    std::string tmp_str;
    
    if (adjust_method == "linear") {
        ROS_INFO("linear mode");
        Eigen::Matrix2d mat;
        // read 4 lines
        getline(infile, tmp_str);
        mat(0, 0) = std::stod(tmp_str);
        getline(infile, tmp_str);
        mat(0, 1) = std::stod(tmp_str);
        getline(infile, tmp_str);
        mat(1, 0) = std::stod(tmp_str);
        getline(infile, tmp_str);
        mat(1, 1) = std::stod(tmp_str);
        infile.close();
        return mat;
    } else if (adjust_method == "non_linear") {
        ROS_INFO("non linear mode");
        Eigen::MatrixXd mat(2,4);
        // read 8 lines
        getline(infile, tmp_str);
        mat(0, 0) = std::stod(tmp_str);
        getline(infile, tmp_str);
        mat(0, 1) = std::stod(tmp_str);
        getline(infile, tmp_str);
        mat(0, 2) = std::stod(tmp_str);
        getline(infile, tmp_str);
        mat(0, 3) = std::stod(tmp_str);
        getline(infile, tmp_str);
        mat(1, 0) = std::stod(tmp_str);
        getline(infile, tmp_str);
        mat(1, 1) = std::stod(tmp_str);
        getline(infile, tmp_str);
        mat(1, 2) = std::stod(tmp_str);
        getline(infile, tmp_str);
        mat(1, 3) = std::stod(tmp_str);
        infile.close();
        return mat;
    } else {
        ROS_INFO_STREAM("this is no such option " << adjust_method);
        Eigen::Matrix2d mat;
        mat(0, 0) = 1;
        mat(0, 1) = 1;
        mat(1, 0) = 1;
        mat(1, 1) = 1;
        return mat;
    }
}

void callBack(const nav_msgs::OdometryConstPtr& msg) {
    ROS_INFO("running ....");
    if (adjust_method == "linear") {
        Eigen::Vector2d vec;
        vec(0) = msg->pose.pose.position.x;
        vec(1) = msg->pose.pose.position.y;
        Eigen::Vector2d result_vec = cal_mat * vec;
        // ROS_INFO_STREAM("vec " << vec);
        // ROS_INFO_STREAM("result vec " << result_vec);
        nav_msgs::Odometry odom;
        odom.child_frame_id = msg->child_frame_id;
        odom.header = msg->header;
        odom.pose.pose.position.x = result_vec(0);
        odom.pose.pose.position.y = result_vec(1);
        odom.twist = msg->twist;
        pub.publish(odom);
    } else if (adjust_method == "non_linear") {
        Eigen::Vector4d vec;
        vec(0) = msg->pose.pose.position.x;
        vec(1) = msg->pose.pose.position.y;
        vec(2) = msg->pose.pose.position.x * msg->pose.pose.position.x;
        vec(3) = msg->pose.pose.position.y * msg->pose.pose.position.y;
        Eigen::Vector2d result_vec = cal_mat * vec;
        nav_msgs::Odometry odom;
        odom.child_frame_id = msg->child_frame_id;
        odom.header = msg->header;
        odom.pose.pose.position.x = result_vec(0);
        odom.pose.pose.position.y = result_vec(1);
        odom.twist = msg->twist;
        pub.publish(odom);
    } else {
        ROS_INFO_STREAM("no method is " << adjust_method);
    }
    

    
}