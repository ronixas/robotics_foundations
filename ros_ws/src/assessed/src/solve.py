#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import rospkg
import tf
import time

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
import moveit_commander

# constants
noOfBlocks = 2
delta = 0.02 # x,y,z center offset
baseName = "block"

class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
	# add listener
	self.listener = tf.TransformListener()
        # let listener buffer to fill before using  
        time.sleep(1)
        
        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        #We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")
        self._group.set_max_velocity_scaling_factor(0.2) # This is to make Baxter move slower

    def move_to_start(self, start_angles=None):

        print("Moving the {0} arm to start pose...".format(self._limb_name))
        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")


    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)


    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def _servo_to_pose(self, pose):
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

    def starting_position(self, start_pose):
        rospy.loginfo("Preparing!")
        self._group.set_pose_target(start_pose)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.loginfo("In position!")


def main():

    # initialize moveit comander and a node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("assessed")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    # decide on the limb (left or right)
    limb = 'left'
    hover_distance = 0.1 # meters

    # create the sorting object
    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    # start positions
    #l_start = Pose(position=Point(x=0.5, y=0.5, z=0.9), orientation=Quaternion(x=0, y=1, z=0, w=0.00486450832011))

    #l_pnp.starting_position(l_start)

    # block poses will be held in a dict indexted with block name
    block_poses = dict()
    block_targets = dict()

    # start orientation
    orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    
    for objNo in range(0, noOfBlocks):
        try:
            objName = baseName + str(objNo)
            # retrieve block transformation with help of a listener
            (trans,rot) = pnp.listener.lookupTransform('/world', '/' + objName, rospy.Time(0))
            # use the transforms to create poses
            start_position = Point(x=trans[0]+delta, y=0, z=float(trans[2])+0.2) # 30cm above the object
            obj_position = Point(x=trans[0]+delta, y=trans[1]+delta, z=trans[2]+delta)

            orientation=Quaternion(x=rot[0], y=1, z=rot[2], w=0.00486450832011)

            # update the block positions
            block_poses[objName] = [Pose(position=start_position, orientation=orientation), Pose(position=obj_position, orientation=orientation)]

	    print "{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}".format(objName, trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]) 

            targ_pos = Point(x=float(trans[0])+delta+0.1, y=trans[1]+delta, z=trans[2]+delta)

            block_targets[objName] = Pose(position=targ_pos, orientation=orientation)

            print objName + " read"
        except Exception as e:
            print e

    # move block one by one
    #while not rospy.is_shutdown():
    for name in block_poses.keys():
            #testing
       print "moving {}".format(name)
            # move over the block
       pnp.move_to_start(block_poses[name][0])
       #print("\nPicking...")
       #print "{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}".format(name, block_poses[name][0].position.x, block_poses[name][0].position.y, block_poses[name][0].position.z, block_poses[name][0].orientation.x, block_poses[name][0].orientation.y, block_poses[name][0].orientation.z, block_poses[name][0].orientation.w)  
       #pnp.pick(block_poses[name][1])
       #print("\nPlacing...")
       #pnp.place(block_targets[name])
    return 0

if __name__ == '__main__':
    sys.exit(main())
