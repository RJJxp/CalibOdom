#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

ros::Publisher pub;
ros::Subscriber sub;
Eigen::MatrixXd mat;


void callBack(const nav_msgs::OdometryConstPtr& msg);
Eigen::MatrixXd readMat(std::string mat_path);
double getYaw(geometry_msgs::Quaternion quat);
// assume both roll and pitch are 0
geometry_msgs::Quaternion yaw2Quaternion(double yaw);

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_result_rjp");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ROS_INFO("start node");

    std::string topic_in, topic_out, mat_path;
    pnh.param("topic_in", topic_in, std::string(""));
    pnh.param("topic_out", topic_out, std::string(""));
    pnh.param("mat_path", mat_path, std::string(""));
    ROS_INFO_STREAM("topic_in is " << topic_in);
    ROS_INFO_STREAM("topic_out is " << topic_out);
    ROS_INFO_STREAM("mat_path is " << mat_path);
    ROS_INFO("the param initialization is done");

    ROS_INFO("start reading mat");
    mat = readMat(mat_path);
    ROS_INFO("read mat successfully, the mat is");
    ROS_INFO_STREAM(mat);

    pub = nh.advertise<nav_msgs::Odometry>(topic_out,1);
    ROS_INFO("pub is done");
    sub = nh.subscribe(topic_in, 1, &callBack);

    ros::spin();

    return 0;
}

Eigen::MatrixXd readMat(std::string mat_path) {
    std::ifstream infile(mat_path, std::ios::in);
    ROS_INFO("start to read the mat");
    std::string tmp_str;
    Eigen::Matrix3d result_mat;

    // read 9 lines
    getline(infile, tmp_str);
    result_mat(0, 0) = std::stod(tmp_str);
    getline(infile, tmp_str);
    result_mat(0, 1) = std::stod(tmp_str);
    getline(infile, tmp_str);
    result_mat(0, 2) = std::stod(tmp_str);
    getline(infile, tmp_str);
    result_mat(1, 0) = std::stod(tmp_str);
    getline(infile, tmp_str);
    result_mat(1, 1) = std::stod(tmp_str);
    getline(infile, tmp_str);
    result_mat(1, 2) = std::stod(tmp_str);
    getline(infile, tmp_str);
    result_mat(2, 0) = std::stod(tmp_str);
    getline(infile, tmp_str);
    result_mat(2, 1) = std::stod(tmp_str);
    getline(infile, tmp_str);
    result_mat(2, 2) = std::stod(tmp_str);

    // ROS_INFO("read mat successfully, the mat is");
    // ROS_INFO_STREAM(result_mat);

    infile.close();
    return result_mat;
}

void callBack(const nav_msgs::OdometryConstPtr& msg) {
    // ROS_INFO("start callback function");
    nav_msgs::Odometry output_msg;
    output_msg.child_frame_id = msg->child_frame_id;
    output_msg.header = msg->header;
    output_msg.twist = msg->twist;
    output_msg.pose.pose.position = msg->pose.pose.position;
    // ROS_INFO("output_msgs frame, header, twist and pose is done");
    // build the vector3d with x, y and yaw
    Eigen::Vector3d input_vec, output_vec;
    double input_raw = getYaw(msg->pose.pose.orientation);
    input_vec(0) = msg->pose.pose.position.x;
    input_vec(1) = msg->pose.pose.position.y;
    input_vec(2) = input_raw;
    // transform to output
    output_vec = mat * input_vec;
    // transform output_yaw to quaternion
    double output_raw = output_vec(2);
    geometry_msgs::Quaternion quat = yaw2Quaternion(output_raw);
    output_msg.pose.pose.orientation = quat;
    // ROS_INFO("output_msgs orientation is done");

    pub.publish(output_msg);
}

double getYaw(geometry_msgs::Quaternion quat) {
    double x, y, z, w;
    x = quat.x;
    y = quat.y;
    z = quat.z;
    w = quat.w;
    Eigen::Quaterniond qqq(x, y, z, w);
    Eigen::Vector3d euler_angle = qqq.toRotationMatrix().eulerAngles(2, 1, 0);
    return euler_angle(2);
}

geometry_msgs::Quaternion yaw2Quaternion(double yaw) {
    double roll = 0;
    double pitch = 0;

    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond quaternion;
    quaternion = yawAngle * pitchAngle * rollAngle;

    geometry_msgs::Quaternion quat;
    quat.x = quaternion.x();
    quat.y = quaternion.y();
    quat.z = quaternion.z();
    quat.w = quaternion.w();
    return quat;
}