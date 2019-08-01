#include <iostream>
#include <fstream>
#include <thread>
#include <math.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

struct MyPose {
    double x,y;
    double th;
};

class CalibOdomTf {
public:
    CalibOdomTf(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle);
    ~CalibOdomTf();
private:
    void callBack(const nav_msgs::OdometryConstPtr& odom_msg, const geometry_msgs::TransformStampedConstPtr& tf_msg);
    double getYaw(geometry_msgs::Quaternion quat);
    void calMat(std::vector<MyPose> op, std::vector<MyPose> tp);
    void writeMat();
private:
    ros::NodeHandle& _nh;
    ros::NodeHandle& _pnh;

    message_filters::Subscriber<nav_msgs::Odometry> *_odom_sub;
    message_filters::Subscriber<geometry_msgs::TransformStamped> *_tf_sub;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::TransformStamped> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy>* _sync;

    std::vector<MyPose> _odom_pose;
    std::vector<MyPose> _tf_pose;
    std::vector<Eigen::MatrixXd> _result_mat;

    std::string _topic_odom_in;
    std::string _topic_tf_in;
    int _cal_sec;
    std::string _mat_output_path;
    bool _filter_angle;
    double _filter_thresold;
    
    int _message_count;
};