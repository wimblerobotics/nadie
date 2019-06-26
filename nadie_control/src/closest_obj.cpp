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
#include <visualization_msgs/Marker.h>
#include <vector>

ros::Publisher marker_pub;


float computeDistance(geometry_msgs::Point32 & p1, geometry_msgs::Point32 & p2) {
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud2_msg) {
    ROS_INFO("--FRAME-- frame_id: %s", cloud2_msg->header.frame_id.c_str());

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
 
    sensor_msgs::PointCloud2 xy_image_cloud;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try{
//        transformStamped = tfBuffer.lookupTransform(cloud2_msg->header.frame_id, "base_link", ros::Time(0), ros::Duration(3.0));
        transformStamped = tfBuffer.lookupTransform("base_link", "base_link", ros::Time(0), ros::Duration(3.0));
       ROS_INFO("transform translation x: %7.3f, y:  %7.3f, z:  %7.3f, rotations x: %7.3f, y: %7.3f, z: %7.3f, w: %7.3f",
                 transformStamped.transform.translation.x,
                 transformStamped.transform.translation.y,
                 transformStamped.transform.translation.z,
                 transformStamped.transform.rotation.x,
                 transformStamped.transform.rotation.y,
                 transformStamped.transform.rotation.z,
                 transformStamped.transform.rotation.w);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
         return;
    }
    tf2::doTransform (*cloud2_msg, xy_image_cloud, transformStamped);
 
    sensor_msgs::PointCloud orig_cloud_msg;
    sensor_msgs::PointCloud cloud_msg;
//    sensor_msgs::convertPointCloud2ToPointCloud(*cloud2_msg, orig_cloud_msg);
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

    //#####static std::vector<visualization_msgs::Marker> prev_markers;
    // for (std::vector<visualization_msgs::Marker>::iterator it = prev_markers.begin(); it != prev_markers.end(); ++it) {
    //     *it.
    // }

    bool deletePreviousMarkers = true;
    for (int i = 0; i < cloud_msg.points.size(); ++i) {
        geometry_msgs::Point32 point = cloud_msg.points[i];
        if (point.z > kMAX_Z) continue; // Ignore obstacles above the robot.

        float dist = computeDistance(fromWhere, point);
        if(dist < lowestDist) {
            lowestDist = dist;
        }

        if (deletePreviousMarkers) {
           visualization_msgs::Marker marker;
            marker.header.frame_id = "d400_depth_optical_frame";
            marker.header.stamp = ros::Time::now();
            marker.ns = "closestNamespace";
            marker.id = i;
            marker.type = shape;
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker.lifetime = ros::Duration();
            marker_pub.publish(marker);
            deletePreviousMarkers = false;
        }

        if (dist < kHAZARD_DISTANCE) {
            // geometry_msgs::Point32 opoint = orig_cloud_msg.points[i];
            // ROS_INFO("orig x: %7.4f, y: %7.4f, z: %7.4f, xlate x: %7.4f, y: %7.4f, z: %7.4f",
            //          opoint.x, opoint.y, opoint.z,
            //          point.x, point.y, point.z);
            visualization_msgs::Marker marker;
            marker.header.frame_id = "d400_depth_optical_frame";
            marker.header.stamp = ros::Time::now();
            marker.ns = "closestNamespace";
            marker.id = i;
            marker.type = shape;
            marker.action = visualization_msgs::Marker::ADD;
   
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = point.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 0.005;
            marker.scale.y = 0.005;
            marker.scale.z = 0.005;

            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration();

            if (point.y > kLEFT_Y) {
                obstacle_left = true;
                left_point = point;
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;
            } else if (point.y < kRIGHT_Y) {
                obstacle_right = true;
                right_point = point;
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0;
            } else {
                obstacle_center = true;
                center_point = point;
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0;
            }

            marker_pub.publish(marker);
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
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/d400/depth/color/points", 1, callback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("closestObjMarker", 1);
    ros::spin();
    return 0;
}