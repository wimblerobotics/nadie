#include <boost/bind.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/Twist.h>
#include <iomanip>      // std::setprecision
#include <iostream>
#include "nadie_control/calibrate.h"
#include <ros/console.h>
#include <string>
#include <sstream>
#include <vector>

Calibrate::Calibrate(ros::NodeHandle& nh)
	: goal_x_(1.0),
      goal_z_(M_PI)
    , last_ficucial_pose_msg_counter_(0)
    , last_fiducials_msg_counter_(0)
    , last_imu_data_msg_counter_(0)
    , last_imu_mag_msg_counter_(0)
    , last_imu_raw_msg_counter_(0)
    , last_imu_status_msg_counter_(0)
    , last_odometry_msg_counter_(0)
    , nh_(nh)
    , prev_(ros::Time::now())
    , start_fiducial_found_(false)
    , start_odometry_found_(false)
    , state_(KSTART) {
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("/nadie/diff_drive_controller/cmd_vel", 1);
    fiducial_pose_subscriber_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/fiducial_pose", 10, boost::bind(&Calibrate::fiducial_pose_callback, this, _1));
    fiducials_subscriber_ = nh.subscribe<visualization_msgs::Marker>("/fiducials", 10, boost::bind(&Calibrate::fiducials_callback, this, _1));
    imu_data_subscriber_ = nh_.subscribe<sensor_msgs::Imu>("imu/data", 10,  boost::bind(&Calibrate::imu_data_callback, this, _1));
    imu_mag_subscriber_ = nh_.subscribe<sensor_msgs::MagneticField>("imu/mag", 10,  boost::bind(&Calibrate::imu_mag_callback, this, _1));
    imu_raw_subscriber_ = nh_.subscribe<sensor_msgs::Imu>("imu/raw", 10, boost::bind(&Calibrate::imu_raw_callback, this, _1));
    imu_status_subscriber_ = nh_.subscribe<diagnostic_msgs::DiagnosticStatus>("imu/status", 10, boost::bind(&Calibrate::imu_status_callback, this, _1));
    odometry_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("diff_drive_controller/odom", 10, boost::bind(&Calibrate::odometry_callback, this, _1));
}


double Calibrate::normalizeRadians(double theta) {
    double TWO_PI = 2 * M_PI;
    double normalized = fmod(theta, TWO_PI);
    normalized = fmod((normalized + TWO_PI), TWO_PI);
    return normalized <= M_PI ? normalized : normalized - TWO_PI;
}

void Calibrate::fiducial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    last_ficucial_pose_msg_ = *msg;
    last_ficucial_pose_msg_counter_++;
    if (!start_fiducial_found_) {
        start_fiducial_ = *msg;
        start_fiducial_found_ = true;
    }
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
    if (!start_odometry_found_) {
        start_odometry_ = *msg;
        start_odometry_found_ = true;
    }
    //ROS_INFO("[Calibrate::odometry_callback] message received counter:: %lu", last_odometry_msg_counter_);
}


std::string Calibrate::eulerString(const sensor_msgs::Imu_<std::allocator<void> >::_orientation_type& q, u_long counter) {
    tf::Quaternion qq(q.x, q.y, q.z, q.w);
    std::stringstream s;
    s << "angle: " << std::setprecision(4);
    if (counter > 0) {
        double yaw = tf::getYaw(qq);
        s << yaw;
        s << "r (" << ((yaw * 360) / (2 * M_PI)) << "d)";
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
    std::string calibStatus = "??";

    if (last_imu_status_msg_counter_ > 0) {
        if (last_imu_status_msg_.name == "BNO055 IMU") {
            std::vector<diagnostic_msgs::KeyValue>::const_iterator jt;
            for (jt = last_imu_status_msg_.values.begin(); jt != last_imu_status_msg_.values.end(); ++jt) {
                if (jt->key == "Calibration status") {
                    std::string::size_type sz;
                    int statusValue = stoi(jt->value, &sz);
                    ROS_INFO("[Calibrate::report] key: %s, value: %s", jt->key.c_str(), jt->value.c_str());//#####
                    std::stringstream st;
                    st << "VAL:" << std::showbase << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << statusValue << std::dec;
                    st << "SYS:" << ((statusValue >> 6) & 0x3);
                    st << "/GYR:" << ((statusValue >> 4) & 0x3);
                    st << "/ACC:" << ((statusValue >> 2) & 0x3);
                    st << "/MAG:" << ((statusValue >> 0) & 0x3);
                    calibStatus = st.str();
                    break;
                }
            }
        }
    }

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

    s << "  ODOM angle: " << eulerString(last_odometry_msg_.pose.pose.orientation, last_odometry_msg_counter_) << std::endl;
    if (last_odometry_msg_counter_ > 0) {
        s << "  ODOM pos x: " << std::setprecision(4) 
        << last_odometry_msg_.pose.pose.position.x 
        << ", y: " 
        << last_odometry_msg_.pose.pose.position.y 
        << ", z: "
        << last_odometry_msg_.pose.pose.position.z;
    } else {
        s << "  ODOM NO DATA";
    }
    s  << std::endl;


    ROS_INFO_STREAM("[Calibrate::report] "  << s.str());
}



bool Calibrate::run() {
    std::string state_string = "????";
    double start_yaw = 0.0;
    switch (state_) {
        case kDONE: state_string = "DONE"; break;
        case kFORWARD: state_string = "kFORWARD"; break;
        case kROTATE_RIGHT: state_string = "kROTATE_RIGHT"; break;
        case KSTART: state_string = "KSTART"; break;
    }

    std::stringstream start_goal_string;
    if (start_odometry_found_ > 0) {
        tf::Quaternion qq(start_odometry_.pose.pose.orientation.x,
                          start_odometry_.pose.pose.orientation.y, 
                          start_odometry_.pose.pose.orientation.z, 
                          start_odometry_.pose.pose.orientation.w);
        start_yaw = tf::getYaw(qq);
        start_goal_string << std::setprecision(4);
        start_goal_string << "START x: " << start_odometry_.pose.pose.position.x;
        start_goal_string << ", y: " << start_odometry_.pose.pose.position.y;
        start_goal_string << ", z: " << start_odometry_.pose.pose.position.z;
        start_goal_string << ", start_yaw: " << start_yaw << "r (" << ((start_yaw * 360) / (2 * M_PI)) << "d)";
    } else {
        start_goal_string << "NO ODOM";
    }

    ROS_INFO_STREAM("[Calibrate::run] state: "
                    << state_string
                    << ", start: " << start_goal_string.str());

    switch (state_) {
        case kDONE:
            return false;
            break;
        
        case kFORWARD:
            if (last_odometry_msg_counter_ > 0) {
                double goal_x = start_odometry_.pose.pose.position.x + goal_x_;
                double goal_x_to_go =  goal_x - last_odometry_msg_.pose.pose.position.x;
                ROS_INFO("[Calibrate::run] kFORWARD goal x: %7.4f, x travel to go %7.4f m", 
                         goal_x,
                         goal_x_to_go);
                if (goal_x_to_go > 0) {
                    ROS_INFO("[Calibrate::run] kFORWARD still need to travel forward");
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0.1;
                    cmd_vel.angular.z = 0.0;
                    cmd_vel_publisher_.publish(cmd_vel);
                } else {
                    // STOP
                    double odometry_x_moved = last_odometry_msg_.pose.pose.position.x - start_odometry_.pose.pose.position.x;
                    double fiducial_x_moved = last_ficucial_pose_msg_.pose.pose.position.x - start_fiducial_.pose.pose.position.x ;
                    state_ = kROTATE_RIGHT;
                    ROS_INFO("[Calibrate::run] kFORWARD goal reached, begin rotation. Odometry distance moved: %7.4f. Fiducial distance moved: %7.4f", odometry_x_moved, fiducial_x_moved);
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    cmd_vel_publisher_.publish(cmd_vel);
                }
            }

            break;
        
        case kROTATE_RIGHT:
            if (last_odometry_msg_counter_ > 0) {
                // ##### Wait on last_ficucial_pose_msg_counter_
                tf::Quaternion qq(last_odometry_msg_.pose.pose.orientation.x,
                                  last_odometry_msg_.pose.pose.orientation.y, 
                                  last_odometry_msg_.pose.pose.orientation.z, 
                                  last_odometry_msg_.pose.pose.orientation.w);
                double current_yaw = tf::getYaw(qq);
                double goal_z = normalizeRadians(start_yaw + goal_z_);
                double goal_z_to_go = normalizeRadians(abs(current_yaw - goal_z));
                ROS_INFO("[Calibrate::run] kROTATE_RIGHT Goal z: %7.4fr (%7.4fd), current z: %7.4fr still need to rotate  %7.4fr (%7.4fd)",
                         goal_z, 
                         ((goal_z * 360) / (2 * M_PI)), 
                         current_yaw,
                         goal_z_to_go, 
                         ((goal_z_to_go * 360) / (2 * M_PI)));
                
                if (goal_z_to_go > 0.04) {
                    ROS_INFO("[Calibrate::run] kROTATE_RIGHT still need to rotate");
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.4;
                    cmd_vel_publisher_.publish(cmd_vel);
                } else {
                    // STOP
                    tf::Quaternion q_start_fiducial_yaw(start_fiducial_.pose.pose.orientation.x,
                                    start_fiducial_.pose.pose.orientation.y, 
                                    start_fiducial_.pose.pose.orientation.z, 
                                    start_fiducial_.pose.pose.orientation.w);
                    double start_fiducial_yaw = tf::getYaw(q_start_fiducial_yaw);
                    tf::Quaternion q_last_fiducial_yaw(last_ficucial_pose_msg_.pose.pose.orientation.x,
                                    last_ficucial_pose_msg_.pose.pose.orientation.y, 
                                    last_ficucial_pose_msg_.pose.pose.orientation.z, 
                                    last_ficucial_pose_msg_.pose.pose.orientation.w);
                    double last_fiducial_yaw = tf::getYaw(q_last_fiducial_yaw);
                    double fiducial_delta = last_fiducial_yaw - start_fiducial_yaw;
                    double odometry_delta = current_yaw - start_yaw;
                   state_ = kDONE;
                    ROS_INFO("[Calibrate::run] kROTATE_RIGHT goal reached, done. Odometry yaw delta: %7.4fr (%7.4fd). Fiducial yaw delta: %7.4fr (%7.4fd)",
                             odometry_delta,
                             ((odometry_delta * 360) / (2 * M_PI)), 
                             fiducial_delta,
                             ((fiducial_delta * 360) / (2 * M_PI)));
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    cmd_vel_publisher_.publish(cmd_vel);
                    report();
                    return false;
                }
            }

            break;
        
        case KSTART:
            if (last_odometry_msg_counter_ > 0) {
                ROS_INFO("[Calibrate::run] Starting. Change goal to kFORWARD");
                state_ = kFORWARD;
            } else {
                ROS_INFO("[Calibrate::run] Starting. Awaiting odometry info");
            }

            break;

        default:
            ROS_INFO("[Calibrate::run] INVALID STATE: %d", state_);
            break;
    }

    report();
    return true;
}


