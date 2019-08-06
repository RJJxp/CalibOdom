#include "../include/calib_odom.h"

CalibOdom::CalibOdom(ros::NodeHandle& node_handle, 
                     ros::NodeHandle& private_node_handle):
                     _nh(node_handle), 
                     _pnh(private_node_handle),
                     _get_result_mat(false) {
    _is_vec_too_large = false;
    _message_count = 0;

    // private variable
    ROS_INFO("start to initialize the private variable");
    _pnh.param("split_or_whole", _split_or_whole, std::string("whole"));
    _pnh.param("adjust_method", _adjust_method, std::string("linear"));
    _pnh.param("cal_sec", _cal_sec, 10);
    _pnh.param("split_length", _split_length, 10);
    _pnh.param("mat_file_path", _mat_file_path, std::string(""));
    ROS_INFO_STREAM("split_or_whole " << _split_or_whole);
    ROS_INFO_STREAM("adjust_method " << _adjust_method);
    ROS_INFO_STREAM("cal_sec " << _cal_sec);
    ROS_INFO_STREAM("split_length " << _split_length);
    ROS_INFO_STREAM("mat_file_path " << _mat_file_path);


    std::string topic_odom, topic_lidar;
    _pnh.param("topic_odom_in", topic_odom, std::string(""));
    _pnh.param("topic_marker_in", topic_lidar, std::string(""));
    ROS_INFO_STREAM("topic_odom_in " << topic_odom);
    ROS_INFO_STREAM("topic_marker_in " << topic_lidar);

    _odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(_nh, topic_odom, 1);
    ROS_INFO("odom sub is done");
    _trajec_sub = new message_filters::Subscriber<visualization_msgs::Marker>(_nh, topic_lidar, 1);
    ROS_INFO("trajec sub is done");
    _sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *_odom_sub, *_trajec_sub);
    ROS_INFO("sync is done");
    _sync->registerCallback(boost::bind(&CalibOdom::callBack, this, _1, _2)); 

    ros::spin();

    if (_split_or_whole == "whole") {
        std::cout << " odom size " << _odom_pose.size() << std::endl;
        std::cout << "lidar pose " << _lidar_pose.size() << std::endl;
        if (_adjust_method == "linear") {
            getResult(_odom_pose, _lidar_pose);
        } else if (_adjust_method == "non_linear") {
            getResultNonLinear(_odom_pose, _lidar_pose);
        } else {
            ROS_INFO("no method found after spin func");
        }
        
    }

    writeMat();
}

void CalibOdom::callBack(const nav_msgs::OdometryConstPtr& odom_msg, 
                         const visualization_msgs::MarkerConstPtr& traject_msg){

    _message_count++;
    // this function is about 25 hz
    if (_message_count >= 25 * _cal_sec) {
        MyPose p1, p2;
        p1.x = odom_msg->pose.pose.position.x;
        p1.y = odom_msg->pose.pose.position.y;
        _odom_pose.push_back(p1);

        int points_size = traject_msg->points.size();
        auto ppp = traject_msg->points[points_size - 1];
        p2.x = ppp.x;
        p2.y = ppp.y;
        // ROS_INFO_STREAM("x " << p1.x - p2.x << "\t" << "y " << p1.y - p2.y);
        _lidar_pose.push_back(p2);

        _message_count = 0;
        
        if (_split_or_whole == "whole") {
            ROS_INFO_STREAM("the pose length is " << _odom_pose.size());
        }
    }

    
    if (_split_or_whole == "split") {
        if (_adjust_method == "linear") {
            if (_odom_pose.size() >= _split_length) {
                getResult(_odom_pose, _lidar_pose);
                _odom_pose.clear();
                _lidar_pose.clear();
            }
        } else if (_adjust_method == "non_linear") {
            if (_odom_pose.size() >= _split_length) {
                getResultNonLinear(_odom_pose, _lidar_pose);
                _odom_pose.clear();
                _lidar_pose.clear();
            }
        } else {
            ROS_INFO_STREAM("there is no method like " << _adjust_method);
        }
        
    }
}

CalibOdom::~CalibOdom() {
    if (_odom_sub) {
        delete _odom_sub;
        _odom_sub = NULL;
    }
    if (_trajec_sub) {
        delete _trajec_sub;
        _trajec_sub = NULL;
    }
    if (_sync) {
        delete _sync;
        _sync = NULL;
    }
}

void CalibOdom::markerCallback(const visualization_msgs::MarkerArrayConstPtr& msg) {
    // ROS_INFO_STREAM(msg->markers.size());
    MyPose tmp;
    int markers_size = msg->markers.size();
    int points_size = msg->markers[markers_size - 1].points.size();
    auto p = msg->markers[markers_size - 1].points[points_size - 1];
    tmp.x = p.x;
    tmp.y = p.y;
    _lidar_pose.push_back(tmp);
}

void CalibOdom::getResult(std::vector<MyPose> odom_pose, 
                          std::vector<MyPose> lidar_pose) {
    // use all the data
    int odom_size = odom_pose.size();
    int lidar_size = lidar_pose.size();
    if (odom_size != lidar_size) {
        ROS_INFO("lidar pose and odom pose size is different");
        _get_result_mat = false;
        return;
    }
    if (odom_size > 100 || lidar_size > 100) {
        ROS_INFO("too many pose");
        _get_result_mat = false;
        return ;
    }
    
    ROS_INFO("linear adjust_mode");
    ROS_INFO("start to calculate matrix");
    Eigen::MatrixXd A_mat(2 * odom_size, 4);
    Eigen::VectorXd b_mat(2 * odom_size);
    for (int i = 0; i < odom_size; i++) {
        // first line
        A_mat(i * 2 + 0, 0) = odom_pose[i].x;
        A_mat(i * 2 + 0, 1) = odom_pose[i].y;
        A_mat(i * 2 + 0, 2) = 0;
        A_mat(i * 2 + 0, 3) = 0;
        // second line
        A_mat(i * 2 + 1, 0) = 0;
        A_mat(i * 2 + 1, 1) = 0;
        A_mat(i * 2 + 1, 2) = odom_pose[i].x;
        A_mat(i * 2 + 1, 3) = odom_pose[i].y;
    }

    for (int i = 0; i < lidar_size; i++) {
        b_mat(i * 2 + 0) = lidar_pose[i].x;
        b_mat(i * 2 + 1) = lidar_pose[i].y;
    }

    
    _result_mat.push_back((A_mat.transpose() * A_mat).inverse() * A_mat.transpose() * b_mat);

    _get_result_mat = true;
    
    if (_split_or_whole == "split") {
        ROS_INFO_STREAM("the result mat of " << _result_mat.size() << "\n" << _result_mat[_result_mat.size() - 1]);
    } else if (_split_or_whole == "whole") {
        std::cout << "the result mat is " << std::endl;
        std::cout << _result_mat[_result_mat.size() - 1] << std::endl;
        ROS_INFO("finished calculating the result mat");
    }
    
}

void CalibOdom::getResultNonLinear(std::vector<MyPose> odom_pose, 
                                   std::vector<MyPose> lidar_pose) {
    int odom_size = odom_pose.size();
    int lidar_size = lidar_pose.size();
    if (odom_size != lidar_size) {
        ROS_INFO("lidar pose and odom pose size is different");
        _get_result_mat = false;
        return;
    }
    if (odom_size > 100 || lidar_size > 100) {
        ROS_INFO("too many pose");
        _get_result_mat = false;
        return ;
    }
    
    ROS_INFO("non_linear adjust_mode");
    ROS_INFO("start to calculate matrix");
    Eigen::MatrixXd A_mat(2 * odom_size, 8);
    Eigen::VectorXd b_mat(2 * odom_size);
    for (int i = 0; i < odom_size; i++) {
        // first line
        A_mat(i * 2 + 0, 0) = odom_pose[i].x;
        A_mat(i * 2 + 0, 1) = odom_pose[i].y;
        A_mat(i * 2 + 0, 2) = odom_pose[i].x * odom_pose[i].x;
        A_mat(i * 2 + 0, 3) = odom_pose[i].y * odom_pose[i].y;
        A_mat(i * 2 + 0, 4) = 0;
        A_mat(i * 2 + 0, 5) = 0;
        A_mat(i * 2 + 0, 6) = 0;
        A_mat(i * 2 + 0, 7) = 0;
        // second line
        A_mat(i * 2 + 1, 0) = 0;
        A_mat(i * 2 + 1, 1) = 0;
        A_mat(i * 2 + 1, 2) = 0;
        A_mat(i * 2 + 1, 3) = 0;
        A_mat(i * 2 + 1, 4) = odom_pose[i].x;
        A_mat(i * 2 + 1, 5) = odom_pose[i].y;
        A_mat(i * 2 + 1, 6) = odom_pose[i].x * odom_pose[i].x;
        A_mat(i * 2 + 1, 7) = odom_pose[i].y * odom_pose[i].y;
    }

    for (int i = 0; i < lidar_size; i++) {
        b_mat(i * 2 + 0) = lidar_pose[i].x;
        b_mat(i * 2 + 1) = lidar_pose[i].y;
    }

    _result_mat.push_back((A_mat.transpose() * A_mat).inverse() * A_mat.transpose() * b_mat);
    _get_result_mat = true;
    
    if (_split_or_whole == "split") {
        ROS_INFO_STREAM("the result mat of " << _result_mat.size() << "\n" << _result_mat[_result_mat.size() - 1]);
    } else if (_split_or_whole == "whole") {
        std::cout << "the result mat is " << std::endl;
        std::cout << _result_mat[_result_mat.size() - 1] << std::endl;
        ROS_INFO("finished calculating the result mat");
    }                                   
}

void CalibOdom::writeMat() {
    std::cout << "start to write result mat " << std::endl;
    std::ofstream outfile(_mat_file_path, std::ios::out);
    for (int i = 0; i < _result_mat.size(); i++) {
        outfile << _result_mat[i];
    }
    outfile.close();
    std::cout << "finished writing mat" << std::endl;

}