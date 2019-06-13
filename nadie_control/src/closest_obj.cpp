#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_listener.h>



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
        transformStamped = tfBuffer.lookupTransform("base_link", cloud2_msg->header.frame_id, ros::Time(0), ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
         return;
    }
    tf2::doTransform (*cloud2_msg, xy_image_cloud, transformStamped);
 
    sensor_msgs::PointCloud cloud_msg;
    sensor_msgs::convertPointCloud2ToPointCloud(xy_image_cloud, cloud_msg);

    geometry_msgs::Point32 fromWhere;
    fromWhere.x = 0.0;
    fromWhere.y = 0.0;
    fromWhere.z = 0.0;
    float lowestDist = 1e8;

    const float kMAX_Z = 0.5; // Max height of robot.
    const float kLEFT_Y = 0.3;
    const float kRIGHT_Y = -0.3;
    const float kHAZARD_DISTANCE = 0.5; // Obstacles closer than this are to be avoided.

    bool obstacle_left = false;
    bool obstacle_center = false;
    bool obstacle_right = false;
    geometry_msgs::Point32 left_point;
    geometry_msgs::Point32 center_point;
    geometry_msgs::Point32 right_point;
    left_point.x = left_point.y = left_point.z = 0.0;
    center_point.x = center_point.y = center_point.z = 0.0;
    right_point.x = right_point.y = right_point.z = 0.0;

    for (int i = 0; i < cloud_msg.points.size(); ++i) {
        geometry_msgs::Point32 point = cloud_msg.points[i];
        if (point.z > kMAX_Z) continue; // Ignore obstacles above the robot.

        float dist = computeDistance(fromWhere, point);
        if(dist < lowestDist) {
            lowestDist = dist;
        }

        if (dist < kHAZARD_DISTANCE) {
            if (point.y > kLEFT_Y) {
                obstacle_left = true;
                left_point = point;
            } else if (point.y < kRIGHT_Y) {
                obstacle_right = true;
                right_point = point;
            } else {
                obstacle_center = true;
                center_point = point;
            }
        }

        //ROS_INFO("point\t%f\t%f\t%f\t%f", cloud_msg.points[i].x, cloud_msg.points[i].y, cloud_msg.points[i].z, dist);
    }

    if (obstacle_center || obstacle_left || obstacle_right) {
        ROS_INFO("HAZARD: left: %s [%f, %f, %f], center: %s [%f, %f, %f], right: %s [%f, %f, %f]"
                    , obstacle_left ? "Y" : "n", left_point.x, left_point.y, left_point.z
                    , obstacle_center ? "Y" : "n", center_point.x, center_point.y, center_point.z
                    , obstacle_right ? "Y" : "n", right_point.x, right_point.y, right_point.z);
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