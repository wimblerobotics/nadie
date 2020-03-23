#!/usr/bin/env python
import rospy
import sys

if __name__ == "__main__":
    rospy.init_node("trace", anonymous = True)
    rospy.loginfo("TRACE: '%s = %s'" % (sys.argv[1], sys.argv[2]))
    print("traceDone")