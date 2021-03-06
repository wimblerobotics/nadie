#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <ros/exception.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_listener.h>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;

static const float kMAX_Z = 0.5; // Max height of robot.
static const float kLEFT_Y = 0.3;
static const float kRIGHT_Y = -0.3;
static const float kHAZARD_DISTANCE = 0.6; // Obstacles closer than this are to be avoided.
static uint32_t shape = visualization_msgs::Marker::SPHERE;
static std_msgs::ColorRGBA kLEFT_COLOR;
static std_msgs::ColorRGBA kRIGHT_COLOR;
static std_msgs::ColorRGBA kCENTER_COLOR;


float computeDistance(geometry_msgs::Point32 & p1, geometry_msgs::Point32 & p2) {
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void addMarker(const geometry_msgs::Point32& point, int id) {
    const float CUBE_SIZE = 0.0254 * 3;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "scan";
    marker.header.stamp = ros::Time::now();
    marker.ns = "closestNamespace";
    marker.id = id;
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
    marker.scale.x = CUBE_SIZE;
    marker.scale.y = CUBE_SIZE;
    marker.scale.z = CUBE_SIZE;

    marker.lifetime = ros::Duration(1.0);

    if (point.y > kLEFT_Y) {
        marker.color = kLEFT_COLOR;
    } else if (point.y < kRIGHT_Y) {
        marker.color = kRIGHT_COLOR;
    } else {
        marker.color = kCENTER_COLOR;
    }

    marker_pub.publish(marker);
}


void callback(const sensor_msgs::PointCloud2ConstPtr& cloud2_msg) {
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
//     geometry_msgs::TransformStamped transformStamped;
//     try{
// //        transformStamped = tfBuffer.lookupTransform(cloud2_msg->header.frame_id, "base_link", ros::Time(0), ros::Duration(3.0));
//         transformStamped = tfBuffer.lookupTransform("base_link", "base_link", ros::Time(0), ros::Duration(3.0));
//        ROS_INFO("transform translation x: %7.3f, y:  %7.3f, z:  %7.3f, rotations x: %7.3f, y: %7.3f, z: %7.3f, w: %7.3f",
//                  transformStamped.transform.translation.x,
//                  transformStamped.transform.translation.y,
//                  transformStamped.transform.translation.z,
//                  transformStamped.transform.rotation.x,
//                  transformStamped.transform.rotation.y,
//                  transformStamped.transform.rotation.z,
//                  transformStamped.transform.rotation.w);
//     }
//     catch (tf2::TransformException &ex) {
//         ROS_WARN("%s",ex.what());
//          return;
//     }

//     sensor_msgs::PointCloud2 xy_image_cloud;
//     tf2::doTransform (*cloud2_msg, xy_image_cloud, transformStamped);
 
    sensor_msgs::PointCloud orig_cloud_msg;
    sensor_msgs::PointCloud cloud_msg;
    sensor_msgs::convertPointCloud2ToPointCloud(*cloud2_msg, cloud_msg);
    // sensor_msgs::convertPointCloud2ToPointCloud(xy_image_cloud, cloud_msg);

    geometry_msgs::Point32 fromWhere;
    fromWhere.x = 0.0;
    fromWhere.y = 0.0;
    fromWhere.z = 0.0;
    float lowestDist = 1e8;


    int left_count = 0;
    int right_count = 0;
    bool obstacle_left = false;
    bool obstacle_center = false;
    bool obstacle_right = false;
    geometry_msgs::Point32 left_point;
    geometry_msgs::Point32 center_point;
    geometry_msgs::Point32 right_point;
    left_point.x = left_point.y = left_point.z = 0.0;
    center_point.x = center_point.y = center_point.z = 0.0;
    right_point.x = right_point.y = right_point.z = 0.0;

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
            if (point.x < 0) left_count += 1;
            else right_count +=1;
            addMarker(point, i);
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
    }

    if (obstacle_center || obstacle_left || obstacle_right) {
        ROS_INFO("left: %d, right: %d, HAZARD: left: %s [%f, %f, %f], center: %s [%f, %f, %f], right: %s [%f, %f, %f]"
                 , left_count, right_count
                    , obstacle_left ? "Y" : "n", left_point.x, left_point.y, left_point.z
                    , obstacle_center ? "Y" : "n", center_point.x, center_point.y, center_point.z
                    , obstacle_right ? "Y" : "n", right_point.x, right_point.y, right_point.z);
    }

    // ROS_INFO("%f\tclosest-point", lowestDist);
}

void laserCallback(const sensor_msgs::LaserScanConstPtr& laser_msg) {
    float lowestDist = 1e8;
    int left_count = 0;
    int right_count = 0;
    bool obstacle_left = false;
    bool obstacle_center = false;
    bool obstacle_right = false;
    geometry_msgs::Point32 left_point;
    geometry_msgs::Point32 center_point;
    geometry_msgs::Point32 right_point;
    left_point.x = left_point.y = left_point.z = 0.0;
    center_point.x = center_point.y = center_point.z = 0.0;
    right_point.x = right_point.y = right_point.z = 0.0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "scan";
    marker.header.stamp = ros::Time::now();
    marker.ns = "closestNamespace";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);

    float angle;
    int i;
    for (i = 0, angle = laser_msg->angle_min; 
         i < laser_msg->ranges.size(); 
         ++i, angle += laser_msg->angle_increment) { 
        float range = laser_msg->ranges[i];
        if ((range < laser_msg->range_min) || (range > laser_msg->range_max)) continue;

        if (range < lowestDist) {
            lowestDist = range;
        }


        if (range < kHAZARD_DISTANCE) {
            geometry_msgs::Point32 marker_point;
            marker_point.x = cos(angle) * range;
            marker_point.y = sin(angle) * range;
            marker_point.z = 0;
            ROS_INFO("angle: %fr (%fd), range: %f, index: %d, x: %f, y: %f",
                     angle,
                     angles::to_degrees(angle),
                     range,
                     i,
                     marker_point.x,
                     marker_point.y);
            addMarker(marker_point, i);
        }
    }
}


void loadURDF(ros::NodeHandle &nh, std::string param_name) {
    std::string urdf_string;
    urdf::Model* model = new urdf::Model();
    ROS_INFO_STREAM("[closest_obj::loadURDF] param_name:"
                    << param_name);
    
    // search and wait for robot_description on param server
    while (urdf_string.empty()) {
        std::string search_param_name;
        if (nh.searchParam(param_name, search_param_name)) {
            ROS_INFO_STREAM("[closest_obj::loadURDF] In namespace: "
                            << nh.getNamespace()
                            << ", waiting for model URDF"
                            " on the ROS param server at search_param_name: "
                            << search_param_name);
            nh.getParam(search_param_name, urdf_string);
            break;
        } else {
            ROS_INFO_STREAM("[closest_obj::loadURDF] In namespace: "
                            << nh.getNamespace()
                            << ", waiting for model URDF"
                            " on the ROS param server at param_name: "
                            << param_name);
            nh.getParam(param_name, urdf_string);
            break;
        }

        usleep(10000);
    }

    ROS_INFO_STREAM("[closest_obj::loadURDF] fetched: " << urdf_string.length() << " bytes");
    ROS_INFO_STREAM("[closest_obj::loadURDF] about to initialize model with string");

    if (!model->initString(urdf_string)) {
        ROS_ERROR_STREAM("[closest_obj::loadURDF] Unable to initialize model with loaded URDF model");
        throw new ros::InvalidParameterException("Unable to initialize model with loaded URDF model");
    } else {
        ROS_INFO_STREAM("[closest_obj::loadURDF] URDF model successfully initialized");
    }


    urdf::LinkConstSharedPtr base_footprint = model->getLink("base_footprint");
    urdf::CollisionConstSharedPtr base_footprint_collision = base_footprint->collision;
}


int main(int argc, char** argv) {
    kLEFT_COLOR.r = 1.0;
    kLEFT_COLOR.g = 1.0;
    kLEFT_COLOR.b = 0.0;
    kLEFT_COLOR.a = 1.0;
    kRIGHT_COLOR.r = 1.0;
    kRIGHT_COLOR.g = 0.0;
    kRIGHT_COLOR.b = 1.0;
    kRIGHT_COLOR.a = 1.0;
    kCENTER_COLOR.r = 0.0;
    kCENTER_COLOR.g = 1.0;
    kCENTER_COLOR.b = 1.0;
    kCENTER_COLOR.a = 1.0;
    
    ros::init(argc, argv, "closest_obj");
    ros::NodeHandle nh;
    loadURDF(nh, "robot_description");
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/nadie/scan", 1, laserCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("closestObjMarker", 1);
    ros::spin();
    return 0;
}