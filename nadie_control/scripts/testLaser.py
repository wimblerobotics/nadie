#! /usr/bin/env python

import os
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations


region_names_ = ['right', 'front_right', 'front', 'front_left', 'left']

state_ = -1
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def clbk_laser(msg):
    max_region_value_ = 30
    number_pixels_ = len(msg.ranges)
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
        new_list_ = [x if x > 0.1 else max_region_value_ for x in selected_data_]
        # rospy.loginfo("============= selected_data: %s", selected_data_)
        # rospy.loginfo("------------ new_list_: %s", new_list_)
        # rospy.loginfo("............ min: %s", min(new_list_))
        
        #rospy.loginfo("selected_data_: %s", selected_data_)
        #rospy.loginfo("max: %s", max(selected_data_))
        #raise SystemExit #####
        regions[region_names_[i]] = min(new_list_)
        if (regions[region_names_[i]] == max_region_value_ ):
            rospy.loginfo("============= region: %s, selected_data: %s", region_names_[i], selected_data_)
            rospy.loginfo("------------ new_list_: %s", new_list_)
            rospy.loginfo("............ min: %s", min(new_list_))
    
    # rospy.loginfo(len(msg.ranges))
    # rospy.loginfo(msg)
    
    #rospy.loginfo("regions: %s", regions)

    # Since we really only want to look at 3 regions, collapse front_left and front to front_left, 
    # and right_right and right to front_right.
    if regions['right'] < regions['front_right']: regions['front_right'] = regions['right']
    if regions['left'] < regions['front_left']: regions['front_left'] = regions['left']
    # rospy.loginfo("\n FINAL REGIONS: %s\n", regions)

    take_action(regions)

def change_state(state, state_description):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower: %s, state[%s]: %s' % (state_description, state, state_dict_[state])
        state_ = state
 
def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''
    
    d = 0.75
    
    if regions['front'] > d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 1 - - - - -'
        change_state(0, state_description)
        rospy.loginfo(" no regious valid: regions: %s", regions)
    elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 2 - - X - -'
        change_state(1, state_description)
    elif regions['front'] > d and regions['front_left'] > d and regions['front_right'] < d:
        state_description = 'case 3 - - - X -'
        change_state(2, state_description)
    elif regions['front'] > d and regions['front_left'] < d and regions['front_right'] > d:
        state_description = 'case 4 - X - - -'
        change_state(0, state_description)
    elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] < d:
        state_description = 'case 5 - - X X -'
        change_state(1, state_description)
    elif regions['front'] < d and regions['front_left'] < d and regions['front_right'] > d:
        state_description = 'case 6 - X X - -'
        change_state(1, state_description)
    elif regions['front'] < d and regions['front_left'] < d and regions['front_right'] < d:
        state_description = 'case 7 - X X X -'
        change_state(1, state_description)
    elif regions['front'] > d and regions['front_left'] < d and regions['front_right'] < d:
        state_description = 'case 8 - X - X -'
        change_state(0, state_description)
    else:
        state_description = '##### UNKNOWN CASE #####'
        rospy.loginfo("%s  -- %s", state_description, regions)
 
def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg
 
def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg
 
def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.5
    return msg
 
def main():
    rospy.init_node('reading_laser')
    pub_ = rospy.Publisher('/nadie/diff_drive_controller/cmd_vel', Twist, queue_size=1)
    sub= rospy.Subscriber("/d435/laser_scan", LaserScan, clbk_laser)

    rate = rospy.Rate(20)
    change_state(0, 'STARTUP')

    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()