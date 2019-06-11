#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_field_conversion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_listener.h>
#include <tgmath.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>



float computeDistance(geometry_msgs::Point32 & p1, geometry_msgs::Point32 & p2) {
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud2_msg) {
    ROS_INFO("--FRAME-- frame_id: %s", cloud2_msg->header.frame_id.c_str());
 
    sensor_msgs::PointCloud2 xy_image_cloud;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("base_link", "map", ros::Time(0), ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
         return;
    }
    tf2::doTransform (*cloud2_msg, xy_image_cloud, transformStamped);
 
    sensor_msgs::PointCloud cloud_msg;
    sensor_msgs::convertPointCloud2ToPointCloud(xy_image_cloud, cloud_msg);

    //  tf::TransformListener listener;
    // ros::Time now = ros::Time::now();
    // listener.waitForTransform("base_link", "map", now, ros::Duration(3.0));
    // ROS_INFO("--post wait--");

    //static tf2_ros::Buffer tf_buffer_;

    geometry_msgs::Point32 fromWhere;
    fromWhere.x = 0.0;
    fromWhere.y = 0.0;
    fromWhere.z = 0.0;
    float lowestDist = 1e8;
    for (int i = 0; i < cloud_msg.points.size(); ++i) {
        float dist = computeDistance(fromWhere, cloud_msg.points[i]);
        if(dist < lowestDist) {
            lowestDist = dist;
        }
        ROS_INFO("point\t%f\t%f\t%f\t%f", cloud_msg.points[i].x, cloud_msg.points[i].y, cloud_msg.points[i].z, dist);
    }

    ROS_INFO("%f\tclosest-point", lowestDist);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "closest_obj");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/rtabmap/cloud_obstacles", 1, callback);
    ros::spin();
    return 0;
}