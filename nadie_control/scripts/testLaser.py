#! /usr/bin/env python

import os
import rospy
from sensor_msgs.msg import LaserScan

region_names_ = ['right', 'front_right', 'front', 'front_left', 'left']

def clbk_laser(msg):
    number_pixels_ = 640
    number_partitions_ = 5
    pixels_per_partition_ = number_pixels_ / number_partitions_
    regions = {
        'right': 100,
        'front_right': 100,
        'front': 100,
        'front_left': 100,
        'left': 100
    }
    for i in range(number_partitions_):
        selector_range_ = range(i * pixels_per_partition_, (i +1) * pixels_per_partition_)
        #rospy.loginfo("i: %s, selector_range_: %s", i, selector_range_)
        selected_data_ = msg.ranges[i * pixels_per_partition_  : (i + 1) * pixels_per_partition_]
        new_list_ = [x if x > 0.1 else 30 for x in selected_data_]
        # rospy.loginfo("============= selected_data: %s", selected_data_)
        # rospy.loginfo("------------ new_list_: %s", new_list_)
        # rospy.loginfo("............ min: %s", min(new_list_))
        
        #rospy.loginfo("selected_data_: %s", selected_data_)
        #rospy.loginfo("max: %s", max(selected_data_))
        #raise SystemExit #####
        regions[region_names_[i]] = min(new_list_)
        if (i == 0):
            rospy.loginfo("============= selected_data: %s", selected_data_)
            rospy.loginfo("------------ new_list_: %s", new_list_)
            rospy.loginfo("............ min: %s", min(new_list_))
    
    # rospy.loginfo(len(msg.ranges))
    # rospy.loginfo(msg)
    rospy.loginfo("regions: %s", regions)

def main():
    rospy.init_node('reading_laser')
    sub= rospy.Subscriber("/d435/laser_scan", LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()