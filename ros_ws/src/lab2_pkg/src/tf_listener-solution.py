#!/usr/bin/env python  
import rospy
import math

# Importing TF to facilitate the task of receiving transformations
import tf2_ros

from transforms3d import quaternions
import numpy as np

if __name__ == '__main__':
    rospy.init_node('baxter_tf_listener')

    # This creates a transform listener object; once created, it starts receiving
    # transformations using the /tf topic and buffers them up for up to 10 seconds.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # This is where the magic happens, a query is passed to the listener for the
    # /base to /head transform by means of the lookupTransform fn. The arguments are
    # from "this frame" to "this frame" at "this specific time"
    # (if you pass "rospy.Time(0), the fn will give you the latest available transform 
    rate = rospy.Rate(1.0)
    
    # **** SOLUTION
    tf_from = ['base', 'base','left_hand','head_camera']
    tf_to = ['head','left_hand','right_hand','left_gripper_base']
    # *************
    
    while not rospy.is_shutdown():
        # **** SOLUTION
        for each_from, each_to in zip(tf_from, tf_to):
            try:
                transformation = tfBuffer.lookup_transform(each_from, each_to, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            rospy.logwarn("Transformation from: " + each_from + " to " + each_to)
            rospy.loginfo("Translation: \n" + str(transformation.transform.translation))
            rospy.loginfo("Quaternion: \n" + str(transformation.transform.rotation))
            print ""
        # *************
        
        rate.sleep()
