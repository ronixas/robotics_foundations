#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates

# This is hard-coded to block for this excercise, yet you can make the script general by adding cmd line arguments
input_basename = "block"
noOfBlocks = 5

# Global variable where the object's pose is stored
pose = None

def get_links_gazebo(link_states_msg):
    # Call back to retrieve the object you are interested in
    global input_basename
    global pose
    poses = {}
    for (link_idx, link_name) in enumerate(link_states_msg.name):
        modelname = link_name.split('::')[0]
        for i in range(0,noOfBlocks):
            if input_basename+str(i) == modelname:
                poses[modelname] = link_states_msg.pose[link_idx]
        
    pose = poses

def main():
    rospy.init_node('gazebo2tfframe')

    # Create TF broadcaster -- this will publish a frame give a pose
    tfBroadcaster = tf.TransformBroadcaster()
    # SUbscribe to Gazebo's topic where all links and objects poses within the simulation are published
    linkStatesSub = rospy.Subscriber('gazebo/link_states', LinkStates, get_links_gazebo)

    rospy.loginfo('Spinning')
    global pose
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if pose is not None:
            for i in range(0,noOfBlocks):
                pos = pose[input_basename+str(i)].position
                ori = pose[input_basename+str(i)].orientation
                rospy.loginfo(pos)
                # Publish transformation given in pose
                tfBroadcaster.sendTransform((pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w), rospy.Time.now(), input_basename+str(i), 'world')
                print "{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}".format(input_basename+str(i), pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)
                rate.sleep()
    
    rospy.spin()

if __name__ == '__main__':
    main()
