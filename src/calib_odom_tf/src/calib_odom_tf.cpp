#include "../include/calib_odom_tf.h"

CalibOdomTf::CalibOdomTf(ros::NodeHandle& node_handle, 
                         ros::NodeHandle& private_node_handle):
                         _nh(node_handle),
                         _pnh(private_node_handle) {
    ROS_INFO("CalibOdomTf initialize ... ");
    _message_count = 0;
    _pnh.param("topic_odom_in", _topic_odom_in, std::string(""));
    _pnh.param("topic_tf_in", _topic_tf_in, std::string(""));
    _pnh.param("mat_output_path", _mat_output_path, std::string(""));
    _pnh.param("cal_sec", _cal_sec, 5);
    _pnh.param("filter_angle", _filter_angle, true);
    _pnh.param("filter_thresold", _filter_thresold, 0.1);
    ROS_INFO_STREAM("topic_odom_in is " << _topic_odom_in);
    ROS_INFO_STREAM("topic_tf_in is " << _topic_tf_in);
    ROS_INFO_STREAM("mat_output_path is " << _mat_output_path);
    ROS_INFO_STREAM("cal_sec is " << _cal_sec);
    ROS_INFO_STREAM("filter angle is " << _filter_angle);
    ROS_INFO_STREAM("filter_thresold is " << _filter_thresold);
    ROS_INFO("initalize all the param");

    _odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(_nh, _topic_odom_in, 1);
    ROS_INFO("odom sub is done");
    _tf_sub = new message_filters::Subscriber<geometry_msgs::TransformStamped>(_nh, _topic_tf_in, 1);
    ROS_INFO("tf sub is done");
    _sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *_odom_sub, *_tf_sub);
    ROS_INFO("sync is done");
    _sync->registerCallback(boost::bind(&CalibOdomTf::callBack, this, _1, _2));
    
    ros::spin();

    calMat(_odom_pose, _tf_pose);
    writeMat();
}

CalibOdomTf::~CalibOdomTf() {
    if (_odom_sub) {
        delete _odom_sub;
        _odom_sub = NULL;
    }
    if (_tf_sub) {
        delete _tf_sub;
        _tf_sub = NULL;
    }
    if (_sync) {
        delete _sync;
        _sync = NULL;
    }
}

void CalibOdomTf::callBack(const nav_msgs::OdometryConstPtr& odom_msg, 
                           const geometry_msgs::TransformStampedConstPtr& tf_msg) {
    // ROS_INFO("callback is running");
    _message_count++;

    if (_message_count >= _cal_sec * 50) {
        MyPose p1, p2;
        p1.x = odom_msg->pose.pose.position.x;
        p1.y = odom_msg->pose.pose.position.y;
        p1.th = getYaw(odom_msg->pose.pose.orientation);

        p2.x = tf_msg->transform.translation.x;
        p2.y = tf_msg->transform.translation.y;
        p2.th = getYaw(tf_msg->transform.rotation);

        // ROS_INFO_STREAM("p1_th " << p1.th);
        // ROS_INFO_STREAM("p2_th " << p2.th);
        // ROS_INFO_STREAM("filter_angle " << _filter_angle);
        // ROS_INFO_STREAM("filter_thresold " << _filter_thresold);
        if (_filter_angle) {
            if (fabs(p1.th - p2.th) > _filter_thresold) {
                ROS_INFO("larger than thresold");
                ROS_INFO("***********************");
                return;
            }
        }

        _odom_pose.push_back(p1);
        _tf_pose.push_back(p2);

        _message_count = 0;
        ROS_INFO_STREAM("pose length is " << _odom_pose.size());
        ROS_INFO("***********************");
    }
}

double CalibOdomTf::getYaw(geometry_msgs::Quaternion quat) {
    double x, y, z, w;
    x = quat.x;
    y = quat.y;
    z = quat.z;
    w = quat.w;
    Eigen::Quaterniond qqq(x, y, z, w);
    Eigen::Vector3d euler_angle = qqq.toRotationMatrix().eulerAngles(2, 1, 0);
    return euler_angle(2);
}

void CalibOdomTf::calMat(std::vector<MyPose> op, std::vector<MyPose> tp) {
    int odom_size = op.size();
    int tf_size = tp.size();
    if (odom_size != tf_size) {
        ROS_INFO("this size is different ");
        return;
    }
    if (odom_size > 200 || tf_size > 200) {
        ROS_INFO("the size of pose is too large");
        return;
    }
    Eigen::MatrixXd A_mat(3 * odom_size, 9);
    Eigen::VectorXd b_mat(3 * odom_size);

    for (int i = 0; i < odom_size; i++) {
        // first line
        A_mat(i * 3 + 0, 0) = op[i].x;
        A_mat(i * 3 + 0, 1) = op[i].y;
        A_mat(i * 3 + 0, 2) = op[i].th;
        A_mat(i * 3 + 0, 3) = 0;
        A_mat(i * 3 + 0, 4) = 0;
        A_mat(i * 3 + 0, 5) = 0;
        A_mat(i * 3 + 0, 6) = 0;
        A_mat(i * 3 + 0, 7) = 0;
        A_mat(i * 3 + 0, 8) = 0;
        // second line
        A_mat(i * 3 + 1, 0) = 0;
        A_mat(i * 3 + 1, 1) = 0;
        A_mat(i * 3 + 1, 2) = 0;
        A_mat(i * 3 + 1, 3) = op[i].x;
        A_mat(i * 3 + 1, 4) = op[i].y;
        A_mat(i * 3 + 1, 5) = op[i].th;
        A_mat(i * 3 + 1, 6) = 0;
        A_mat(i * 3 + 1, 7) = 0;
        A_mat(i * 3 + 1, 8) = 0;
        // third line
        A_mat(i * 3 + 2, 0) = 0;
        A_mat(i * 3 + 2, 1) = 0;
        A_mat(i * 3 + 2, 2) = 0;
        A_mat(i * 3 + 2, 3) = 0;
        A_mat(i * 3 + 2, 4) = 0;
        A_mat(i * 3 + 2, 5) = 0;
        A_mat(i * 3 + 2, 6) = op[i].x;
        A_mat(i * 3 + 2, 7) = op[i].y;
        A_mat(i * 3 + 2, 8) = op[i].th;
    }
    for (int i = 0; i < tf_size; i++) {
        b_mat(i * 3 + 0) = tp[i].x;
        b_mat(i * 3 + 1) = tp[i].y;
        b_mat(i * 3 + 2) = tp[i].th;
    }
    std::cout << "finished construction of A_mat, b_mat" << std::endl;
    _result_mat.push_back((A_mat.transpose() * A_mat).inverse() * A_mat.transpose() * b_mat);
    std::cout << "the result mat is " << std::endl;
    std::cout << _result_mat[_result_mat.size() - 1] << std::endl;
}

void CalibOdomTf::writeMat() {
    std::cout << "start to write mat to " << std::endl;
    std::cout << _mat_output_path << std::endl;
    std::ofstream outfile(_mat_output_path);
    for (int i = 0; i < _result_mat.size(); i++) {
        outfile << _result_mat[i] << std::endl;
    }
    outfile.close();
    std::cout << "finished writting" << std::endl;
}