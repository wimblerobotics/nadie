#ifndef __CALIBRATE__
#define __CALIBRATE__

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

class Calibrate {
public:
 Calibrate(ros::NodeHandle& nh);

	bool init();

	void run();

private:
	// ROS Parameters.

	// ROS variables;
	ros::NodeHandle& nh_;					// ROS node handle;
	ros::Time prev_;						// Previous loop time, for computing durations.
 
    ros::Subscriber fiducial_pose_subscriber_;  // Subscriber to geometry_msgs/PoseWithCovarianceStamped
    ros::Subscriber fiducials_subscriber_;      // Subscriber to visualization_msgs/Marker
    ros::Subscriber imu_data_subscriber_;       // Subscriber to imu/data message.
    ros::Subscriber imu_mag_subscriber_;        // Subscriber to imu/mag message.
    ros::Subscriber imu_raw_subscriber_;        // Subscriber to imu/raw message.
    ros::Subscriber imu_status_subscriber_;     // Subscriber to imu/status message.
    ros::Subscriber odometry_subscriber_;       // Subscriber to nav_msgs/odometry message.
    
    u_long last_ficucial_pose_msg_counter_;
    geometry_msgs::PoseWithCovarianceStamped last_ficucial_pose_msg_;
    u_long last_fiducials_msg_counter_;
    visualization_msgs::Marker last_fiducials_msg_;
    u_long last_imu_data_msg_counter_;
    sensor_msgs::Imu last_imu_data_msg_;
    u_long last_imu_mag_msg_counter_;
    sensor_msgs::MagneticField last_imu_mag_msg_;
    u_long last_imu_raw_msg_counter_;
    sensor_msgs::Imu last_imu_raw_msg_;
    u_long last_imu_status_msg_counter_;
    diagnostic_msgs::DiagnosticStatus last_imu_status_msg_;
    u_long last_odometry_msg_counter_;
    nav_msgs::Odometry last_odometry_msg_;

    // Functions.
    void fiducial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void fiducials_callback(const visualization_msgs::MarkerConstPtr& msg);
    void imu_data_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void imu_mag_callback(const sensor_msgs::MagneticField::ConstPtr& msg);
    void imu_raw_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void imu_status_callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);

    std::string eulerString(const sensor_msgs::Imu_<std::allocator<void> >::_orientation_type& q, u_long counter);
    void report();

};

#endif
