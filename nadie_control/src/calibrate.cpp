#include <boost/bind.hpp>
#include <ros/console.h>
#include <iomanip>      // std::setprecision
#include <iostream>
#include "nadie_control/calibrate.h"
#include <string>
#include <sstream>

Calibrate::Calibrate(ros::NodeHandle& nh)
	: last_ficucial_pose_msg_counter_(0)
    , last_fiducials_msg_counter_(0)
    , last_imu_data_msg_counter_(0)
    , last_imu_mag_msg_counter_(0)
    , last_imu_raw_msg_counter_(0)
    , last_imu_status_msg_counter_(0)
    , last_odometry_msg_counter_(0)
    , nh_(nh)
    , prev_(ros::Time::now()) {
    fiducial_pose_subscriber_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/fiducial_pose", 10, boost::bind(&Calibrate::fiducial_pose_callback, this, _1));
    fiducials_subscriber_ = nh.subscribe<visualization_msgs::Marker>("/fiducials", 10, boost::bind(&Calibrate::fiducials_callback, this, _1));
    imu_data_subscriber_ = nh_.subscribe<sensor_msgs::Imu>("imu/data", 10,  boost::bind(&Calibrate::imu_data_callback, this, _1));
    imu_mag_subscriber_ = nh_.subscribe<sensor_msgs::MagneticField>("imu/mag", 10,  boost::bind(&Calibrate::imu_mag_callback, this, _1));
    imu_raw_subscriber_ = nh_.subscribe<sensor_msgs::Imu>("imu/raw", 10, boost::bind(&Calibrate::imu_raw_callback, this, _1));
    imu_status_subscriber_ = nh_.subscribe<diagnostic_msgs::DiagnosticStatus>("imu/status", 10, boost::bind(&Calibrate::imu_status_callback, this, _1));
    odometry_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("diff_drive_controller/odom", 10, boost::bind(&Calibrate::odometry_callback, this, _1));
}

void Calibrate::fiducial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    last_ficucial_pose_msg_ = *msg;
    last_ficucial_pose_msg_counter_++;

    //ROS_INFO("[Calibrate::fiducial_pose_callback] message received counter:: %lu", last_ficucial_pose_msg_counter_);

}

void Calibrate::fiducials_callback(const visualization_msgs::MarkerConstPtr& msg) {
    last_fiducials_msg_ = *msg;
    last_fiducials_msg_counter_++;


    //ROS_INFO("[Calibrate::fiducials_callback] message received counter:: %lu", last_fiducials_msg_counter_);
}


void Calibrate::imu_data_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    last_imu_data_msg_ = *msg;
    last_imu_data_msg_counter_++;

    //ROS_INFO("[Calibrate::imu_data_callback] message received counter:: %lu", last_imu_data_msg_counter_);
}


void Calibrate::imu_mag_callback(const sensor_msgs::MagneticField::ConstPtr& msg) {
    last_imu_mag_msg_ = *msg;
    last_imu_mag_msg_counter_++;
    //ROS_INFO("[Calibrate::imu_mag_callback] message received counter:: %lu", last_imu_mag_msg_counter_);
}


void Calibrate::imu_raw_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    last_imu_raw_msg_ = *msg;
    last_imu_raw_msg_counter_++;
    //ROS_INFO("[Calibrate::imu_raw_callback] message received counter:: %lu", last_imu_raw_msg_counter_);
}


void Calibrate::imu_status_callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg) {
    last_imu_status_msg_ = *msg;
    last_imu_status_msg_counter_++;
    //ROS_INFO("[Calibrate::imu_status_callback] message received counter:: %lu", last_imu_status_msg_counter_);
}


bool Calibrate::init() {
    ROS_INFO("[Calibrate::init]");
    return true;
}

void Calibrate::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    last_odometry_msg_ = *msg;
    last_odometry_msg_counter_++;

    //ROS_INFO("[Calibrate::odometry_callback] message received counter:: %lu", last_odometry_msg_counter_);
}


std::string Calibrate::eulerString(const sensor_msgs::Imu_<std::allocator<void> >::_orientation_type& q, u_long counter) {
    tf::Quaternion qq(q.x, q.y, q.z, q.w);
    std::stringstream s;
    s << "angle: " << std::setprecision(2);
    if (counter > 0) {
        s << tf::getYaw(qq);

        s << std::setprecision(4);
        s << ", QUAT x: " << q.x;
        s << ", y: " << q.y;
        s << ", z: " << q.z;
        s << ", w: " << q.w;
    } else {
        s << "NO DATA";
    }
    
    return s.str();
}


void Calibrate::report() {
    std::stringstream s;
    s << std::endl << "IMU calib status: ";
    if (last_imu_status_msg_counter_ > 0) {
        //s << last_imu_status_msg_.??? ;
    } else {
        s << "??";
    }

    s << std::endl;

    s << "  fiducial angle: " << eulerString(last_ficucial_pose_msg_.pose.pose.orientation, last_ficucial_pose_msg_counter_) << std::endl;
    
    if (last_ficucial_pose_msg_counter_ > 0) {
        s << "  fiducial position: x: " << std::setprecision(2) << last_ficucial_pose_msg_.pose.pose.position.x;
        s << ", y: " << last_ficucial_pose_msg_.pose.pose.position.y;
        s << ", z: " << last_ficucial_pose_msg_.pose.pose.position.z;
    } else {
        s << "  fiducial position: NO DATA";
    }

    s << std::endl;

    s << "  IMU data angle: " << eulerString(last_imu_data_msg_.orientation, last_imu_data_msg_counter_) << std::endl;

    s << "  IMU raw angle: " << eulerString(last_imu_raw_msg_.orientation, last_imu_raw_msg_counter_) << std::endl;

    if (last_imu_mag_msg_counter_ > 0) {
        s << "  IMU mag: x: " << last_imu_mag_msg_.magnetic_field.x;
        s << "  , y: " << last_imu_mag_msg_.magnetic_field.y;
        s << "  , z: " << last_imu_mag_msg_.magnetic_field.z;
    } else {
        s << "  IMU mag: NO DATA";
    }
    s << std::endl;


    ROS_INFO_STREAM("[Calibrate::report] "  << s.str());
}



void Calibrate::run() {
    report();
}


