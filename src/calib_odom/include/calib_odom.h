#include <iostream>
#include <thread>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

struct MyPose {
    double x, y;
    // double th;
};

class CalibOdom {
public:
    CalibOdom(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle);
    ~CalibOdom();

private:
    void writeMat();
    void callBack(const nav_msgs::OdometryConstPtr& odom_msg, const visualization_msgs::MarkerConstPtr& traject_msg);
    void markerCallback(const visualization_msgs::MarkerArrayConstPtr& msg);
    void getResult(std::vector<MyPose> odom_pose, 
                   std::vector<MyPose> lidar_pose);
    void getResultNonLinear(std::vector<MyPose> odom_pose, 
                            std::vector<MyPose> lidar_pose);

    ros::NodeHandle& _nh;
    ros::NodeHandle& _pnh;

    message_filters::Subscriber<nav_msgs::Odometry> *_odom_sub;
    message_filters::Subscriber<visualization_msgs::Marker> *_trajec_sub;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, visualization_msgs::Marker> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> *_sync;
    
    ros::Subscriber _markerSub;
    
    std::vector<MyPose> _odom_pose;
    std::vector<MyPose> _lidar_pose;
    std::vector<Eigen::MatrixXd > _result_mat;

    std::string _split_or_whole;
    std::string _adjust_method;
    int _cal_sec;
    int _split_length;
    std::string _mat_file_path;

    bool _get_result_mat;
    bool _is_vec_too_large;
    int _message_count;

};